///|/ Copyright (c) Prusa Research 2016 - 2023 Vojtěch Bubník @bubnikv, Pavel Mikuš @Godrak, Lukáš Matěna @lukasmatena, Lukáš Hejl @hejllukas
///|/ Copyright (c) Slic3r 2014 - 2016 Alessandro Ranellucci @alranel
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#include <boost/log/trivial.hpp>
#include <atomic>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <mutex>
#include <algorithm>
#include <string>
#include <map>
#include <array>
#include <cmath>
#include <initializer_list>
#include <iterator>
#include <utility>
#include <vector>
#include <cstddef>

#include "ExPolygon.hpp"
#include "Flow.hpp"
#include "Layer.hpp"
#include "BridgeDetector.hpp"
#include "ClipperUtils.hpp"
#include "Geometry.hpp"
#include "PerimeterGenerator.hpp"
#include "Print.hpp"
#include "Surface.hpp"
#include "BoundingBox.hpp"
#include "SVG.hpp"
#include "Algorithm/RegionExpansion.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/MultiMaterialSegmentation.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Polyline.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/Utils.hpp"
#include "libslic3r/libslic3r.h"
#include "LayerRegion.hpp"

namespace Slic3r {

Flow LayerRegion::flow(FlowRole role) const
{
    return this->flow(role, m_layer->height);
}

Flow LayerRegion::flow(FlowRole role, double layer_height) const
{
    return m_region->flow(*m_layer->object(), role, layer_height, m_layer->id() == 0);
}

Flow LayerRegion::bridging_flow(FlowRole role, bool force_thick_bridges) const
{
    const PrintRegion       &region         = this->region();
    const PrintRegionConfig &region_config  = region.config();
    const PrintObject       &print_object   = *this->layer()->object();
    if (print_object.config().thick_bridges || force_thick_bridges) {
        // The old Slic3r way (different from all other slicers): Use rounded extrusions.
        // Get the configured nozzle_diameter for the extruder associated to the flow role requested.
        // Here this->extruder(role) - 1 may underflow to MAX_INT, but then the get_at() will follback to zero'th element, so everything is all right.
        auto nozzle_diameter = float(print_object.print()->config().nozzle_diameter.get_at(region.extruder(role) - 1));
        // Applies default bridge spacing.
        return Flow::bridging_flow(float(sqrt(region_config.bridge_flow_ratio)) * nozzle_diameter, nozzle_diameter);
    } else {
        // The same way as other slicers: Use normal extrusions. Apply bridge_flow_ratio while maintaining the original spacing.
        return this->flow(role).with_flow_ratio(region_config.bridge_flow_ratio);
    }
}

// Fill in layerm->fill_surfaces by trimming the layerm->slices by layerm->fill_expolygons.
void LayerRegion::slices_to_fill_surfaces_clipped()
{
    // Collect polygons per surface type.
    std::array<std::vector<const Surface*>, size_t(stCount)> by_surface;
    for (const Surface &surface : this->slices())
        by_surface[size_t(surface.surface_type)].emplace_back(&surface);
    // Trim surfaces by the fill_boundaries.
    m_fill_surfaces.surfaces.clear();
    for (size_t surface_type = 0; surface_type < size_t(stCount); ++ surface_type) {
        const std::vector<const Surface*> &this_surfaces = by_surface[surface_type];
        if (! this_surfaces.empty())
            m_fill_surfaces.append(intersection_ex(this_surfaces, this->fill_expolygons()), SurfaceType(surface_type));
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// External perimeter resolution: statistics, debug dumps and the high-definition external runs.
// ---------------------------------------------------------------------------------------------------------------------

namespace {

struct ExtResStats {
    std::atomic<size_t> islands_total { 0 };
    std::atomic<size_t> islands_processed { 0 };
    std::atomic<size_t> fallback_mapping { 0 };        // not exactly one sub-expolygon maps to the island (for some plane)
    std::atomic<size_t> fallback_holes { 0 };          // hole count differs
    std::atomic<size_t> fallback_counts { 0 };         // external entity / closed loop / clockwise loop counts differ
    std::atomic<size_t> fallback_no_externals { 0 };   // the normal run produced no external loop
    std::atomic<size_t> fallback_layer_unsupported { 0 };   // multi-region layers, PerimeterRegions
    std::atomic<size_t> sub_runs { 0 };
};

std::mutex                                       g_ext_res_mutex;
std::map<const PrintObject*, ExtResStats>        g_ext_res_stats;

ExtResStats& ext_res_stats(const PrintObject *object)
{
    std::lock_guard<std::mutex> lock(g_ext_res_mutex);
    return g_ext_res_stats[object];     // std::map: references stay valid
}

const char* ext_res_dump_dir()
{
    static const char *dir = std::getenv("SLIC3R_DUMP_EXTERNAL_SUBPASSES");
    return dir != nullptr && *dir != 0 ? dir : nullptr;
}

bool is_external_entity(const ExtrusionEntity *entity)
{
    if (auto *loop = dynamic_cast<const ExtrusionLoop*>(entity))
        return ! loop->paths.empty() && loop->paths.front().attributes().perimeter_index.has_value() && *loop->paths.front().attributes().perimeter_index == 0;
    if (auto *mp = dynamic_cast<const ExtrusionMultiPath*>(entity))
        return ! mp->paths.empty() && mp->paths.front().attributes().perimeter_index.has_value() && *mp->paths.front().attributes().perimeter_index == 0;
    return false;
}

struct ExternalSignature {
    size_t n_external { 0 };
    size_t n_closed   { 0 };
    size_t n_clockwise{ 0 };
    bool operator==(const ExternalSignature &rhs) const { return n_external == rhs.n_external && n_closed == rhs.n_closed && n_clockwise == rhs.n_clockwise; }
};

ExternalSignature external_signature(const std::vector<const ExtrusionEntity*> &externals)
{
    ExternalSignature sig;
    for (const ExtrusionEntity *e : externals) {
        ++ sig.n_external;
        if (auto *loop = dynamic_cast<const ExtrusionLoop*>(e)) {
            ++ sig.n_closed;
            if (loop->is_clockwise())
                ++ sig.n_clockwise;
        }
    }
    return sig;
}

Point entity_center(const ExtrusionEntity *e)
{
    Points pts;
    e->collect_points(pts);
    return get_extents(pts).center();
}

// Entity kind for matching: 0 = open (multipath), 1 = closed counter-clockwise, 2 = closed clockwise.
int entity_kind(const ExtrusionEntity *e)
{
    if (auto *loop = dynamic_cast<const ExtrusionLoop*>(e))
        return loop->is_clockwise() ? 2 : 1;
    return 0;
}

// Order the new externals so that they follow the slots of the normal externals (greedy by kind and distance of the bounding box centers).
// Returns false when the matching is impossible.
bool match_externals_to_slots(const std::vector<const ExtrusionEntity*> &slots, const std::vector<ExtrusionEntity*> &fresh, std::vector<ExtrusionEntity*> &ordered)
{
    if (slots.size() != fresh.size())
        return false;
    std::vector<Point> fresh_centers;
    fresh_centers.reserve(fresh.size());
    for (const ExtrusionEntity *e : fresh)
        fresh_centers.emplace_back(entity_center(e));
    std::vector<char> used(fresh.size(), 0);
    ordered.assign(slots.size(), nullptr);
    for (size_t i = 0; i < slots.size(); ++ i) {
        const Point c    = entity_center(slots[i]);
        const int   kind = entity_kind(slots[i]);
        double best = std::numeric_limits<double>::max();
        size_t best_j = size_t(-1);
        for (size_t j = 0; j < fresh.size(); ++ j)
            if (! used[j] && entity_kind(fresh[j]) == kind) {
                const double d = (fresh_centers[j] - c).cast<double>().squaredNorm();
                if (d < best) { best = d; best_j = j; }
            }
        if (best_j == size_t(-1))
            return false;
        used[best_j] = 1;
        ordered[i] = fresh[best_j];
    }
    return true;
}

// Debug dump of external loops (text, mm): one line per entity.
void ext_res_append_text(std::string &out, const ExtrusionEntity *e)
{
    Points pts;
    e->collect_points(pts);
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%c %d %d", dynamic_cast<const ExtrusionLoop*>(e) ? 'L' : 'M', int(dynamic_cast<const ExtrusionLoop*>(e) != nullptr),
        int(dynamic_cast<const ExtrusionLoop*>(e) != nullptr && static_cast<const ExtrusionLoop*>(e)->is_clockwise()));
    out += buf;
    for (const Point &p : pts) {
        std::snprintf(buf, sizeof(buf), " %.6f %.6f", unscale<double>(p.x()), unscale<double>(p.y()));
        out += buf;
    }
    out += '\n';
}

void ext_res_append_expolygons(std::string &out, const ExPolygons &expolygons)
{
    char buf[96];
    for (const ExPolygon &ex : expolygons)
        for (size_t i = 0; i < ex.num_contours(); ++ i) {
            out += i == 0 ? "S 1 0" : "S 0 0";
            for (const Point &p : ex.contour_or_hole(i).points) {
                std::snprintf(buf, sizeof(buf), " %.6f %.6f", unscale<double>(p.x()), unscale<double>(p.y()));
                out += buf;
            }
            out += '\n';
        }
}

void ext_res_write_file(const std::string &name, const std::string &text)
{
    std::ofstream f(std::string(ext_res_dump_dir()) + "/" + name, std::ios::app);
    f << text;
}

std::string ext_res_plane_name(const char *prefix, double z)
{
    char buf[64];
    std::snprintf(buf, sizeof(buf), "%s_%.4f.txt", prefix, z);
    return buf;
}

void collect_external_entities(const ExtrusionEntityCollection &perimeters, std::vector<const ExtrusionEntity*> &out)
{
    for (const ExtrusionEntity *island : perimeters.entities)
        if (island->is_collection())
            for (const ExtrusionEntity *e : static_cast<const ExtrusionEntityCollection*>(island)->entities)
                if (is_external_entity(e))
                    out.emplace_back(e);
}

// Plain (k == 1) externals of a layer, tagged by the slicing plane: used to compare with the sub-passes of a thick run.
void ext_res_dump_plain_externals(const LayerRegion &layerm, const ExtrusionEntityCollection &perimeters)
{
    std::vector<const ExtrusionEntity*> externals;
    collect_external_entities(perimeters, externals);
    std::string text;
    for (const ExtrusionEntity *e : externals)
        ext_res_append_text(text, e);
    ExPolygons input;
    for (const Surface &s : layerm.slices())
        input.emplace_back(s.expolygon);
    ext_res_append_expolygons(text, input);
    ext_res_write_file(ext_res_plane_name("plane", layerm.layer()->slice_z), text);
}

} // namespace

// Produce perimeter extrusions, gap fill extrusions and fill polygons for input slices.
void LayerRegion::make_perimeters(
    // Input slices for which the perimeters, gap fills and fill expolygons are to be generated.
    const SurfaceCollection                                &slices,
    // Configuration regions that will be applied to parts of created perimeters.
    const PerimeterRegions                                 &perimeter_regions,
    // Ranges of perimeter extrusions and gap fill extrusions per suface, referencing
    // newly created extrusions stored at this LayerRegion.
    std::vector<std::pair<ExtrusionRange, ExtrusionRange>> &perimeter_and_gapfill_ranges,
    // All fill areas produced for all input slices above.
    ExPolygons                                             &fill_expolygons,
    // Ranges of fill areas above per input slice.
    std::vector<ExPolygonRange>                            &fill_expolygons_ranges)
{
    m_perimeters.clear();
    m_thin_fills.clear();
    m_external_sub_passes.clear();

    perimeter_and_gapfill_ranges.reserve(perimeter_and_gapfill_ranges.size() + slices.size());
    // There may be more expolygons produced per slice, thus this reserve is conservative.
    fill_expolygons.reserve(fill_expolygons.size() + slices.size());
    fill_expolygons_ranges.reserve(fill_expolygons_ranges.size() + slices.size());

    const PrintConfig       &print_config  = this->layer()->object()->print()->config();
    const PrintRegionConfig &region_config = this->region().config();
    // This needs to be in sync with PrintObject::_slice() slicing_mode_normal_below_layer!
    bool spiral_vase = print_config.spiral_vase &&
        //FIXME account for raft layers.
        (this->layer()->id() >= size_t(region_config.bottom_solid_layers.value) &&
         this->layer()->print_z >= region_config.bottom_solid_min_thickness - EPSILON);

    PerimeterGenerator::Parameters params(
        this->layer()->height,
        int(this->layer()->id()),
        this->flow(frPerimeter),
        this->flow(frExternalPerimeter),
        this->flow(frFirstInternalPerimeter),
        this->flow(frSecondInternalPerimeter),
        this->bridging_flow(frPerimeter),
        this->flow(frSolidInfill),
        region_config,
        this->layer()->object()->config(),
        print_config,
        perimeter_regions,
        spiral_vase
    );
    // The two-pass bridge split cannot coexist with the level schedule of the external perimeter resolution (MVP).
    params.two_pass_perimeters = this->layer()->external_resolution() <= 1;

    // Cummulative sum of polygons over all the regions.
    const ExPolygons *lower_slices = this->layer()->lower_layer ? &this->layer()->lower_layer->lslices : nullptr;
    const ExPolygons *upper_slices = this->layer()->upper_layer ? &this->layer()->upper_layer->lslices : nullptr;
    // Cache for offsetted lower_slices
    Polygons          lower_layer_polygons_cache;

    for (const Surface &surface : slices) {
        auto perimeters_begin      = uint32_t(m_perimeters.size());
        auto gap_fills_begin       = uint32_t(m_thin_fills.size());
        auto fill_expolygons_begin = uint32_t(fill_expolygons.size());
        if (this->layer()->object()->config().perimeter_generator.value == PerimeterGeneratorType::Arachne && !spiral_vase)
            PerimeterGenerator::process_arachne(
                // input:
                params,
                surface,
                lower_slices,
                upper_slices,
                lower_layer_polygons_cache,
                // output:
                m_perimeters,
                m_thin_fills,
                fill_expolygons);
        else
            PerimeterGenerator::process_classic(
                // input:
                params,
                surface,
                lower_slices,
                upper_slices,
                lower_layer_polygons_cache,
                // output:
                m_perimeters,
                m_thin_fills,
                fill_expolygons);
        perimeter_and_gapfill_ranges.emplace_back(
            ExtrusionRange{ perimeters_begin, uint32_t(m_perimeters.size()) }, 
            ExtrusionRange{ gap_fills_begin,  uint32_t(m_thin_fills.size()) });
        fill_expolygons_ranges.emplace_back(ExtrusionRange{ fill_expolygons_begin, uint32_t(fill_expolygons.size()) });
    }

    // External perimeter resolution. Only the single LayerRegion layers without PerimeterRegions are supported for now.
    // TODO: layers merging several LayerRegions (slices merged from their sub-slices) and layers with PerimeterRegions fall back to k = 1.
    if (this->layer()->external_resolution() > 1 && ! m_sub_slices.empty() && ! spiral_vase && region_config.perimeters > 0 &&
        this->layer()->object()->config().perimeter_generator.value == PerimeterGeneratorType::Arachne) {
        if (&slices == &m_slices && perimeter_regions.empty())
            this->make_external_sub_passes(params, slices, perimeter_and_gapfill_ranges);
        else
            ext_res_stats(this->layer()->object()).fallback_layer_unsupported += slices.size();
    } else if (ext_res_dump_dir() != nullptr && region_config.perimeters > 0)
        ext_res_dump_plain_externals(*this, m_perimeters);
}


void external_resolution_stats_reset(const PrintObject *object)
{
    std::lock_guard<std::mutex> lock(g_ext_res_mutex);
    g_ext_res_stats.erase(object);
}

void external_resolution_stats_log(const PrintObject *object, double perimeter_stage_seconds)
{
    size_t layers_k = 0;
    for (const Layer *layer : object->layers())
        if (layer->external_resolution() > 1)
            ++ layers_k;
    if (layers_k == 0)
        return;
    const ExtResStats &s = ext_res_stats(object);
    BOOST_LOG_TRIVIAL(debug) << "External resolution: layers with k > 1: " << layers_k << " of " << object->layer_count()
        << ", islands " << s.islands_total.load() << ", processed " << s.islands_processed.load() << ", sub-runs " << s.sub_runs.load()
        << ", fallbacks: mapping " << s.fallback_mapping.load() << ", holes " << s.fallback_holes.load() << ", counts " << s.fallback_counts.load()
        << ", no externals " << s.fallback_no_externals.load() << ", unsupported layer (multi-region / perimeter regions; surfaces) " << s.fallback_layer_unsupported.load()
        << ", perimeter stage " << perimeter_stage_seconds << " s";
}

// External perimeter resolution: for every island of this region with k > 1 run the Arachne generator on the sub-slices (flows at the pass height h/k),
// keep only the external loops, replace the normal external loops of the island by the top pass (k-1) and store the lower passes.
void LayerRegion::make_external_sub_passes(
    const PerimeterGenerator::Parameters                         &params,
    const SurfaceCollection                                      &slices,
    const std::vector<std::pair<ExtrusionRange, ExtrusionRange>> &perimeter_and_gapfill_ranges)
{
    const Layer *layer = this->layer();
    const size_t k     = layer->external_resolution();
    assert(k > 1 && m_sub_slices.size() == k && &slices == &m_slices);
    assert(perimeter_and_gapfill_ranges.size() >= slices.size());
    ExtResStats &stats = ext_res_stats(layer->object());
    const size_t range_offset = perimeter_and_gapfill_ranges.size() - slices.size();
    const double h_pass       = layer->height / double(k);

    // Parameters of the sub-runs: flows at the pass height, no extra perimeters on overhangs, no two-pass bridge split.
    PrintRegionConfig sub_config = params.config;
    sub_config.extra_perimeters_on_overhangs.value = false;
    const PrintRegionConfig &region_config = this->region().config();
    const Flow ext_flow_pass = this->flow(frExternalPerimeter, h_pass);
    // Overhang flow at the pass height: with thick bridges (round strand independent of the layer height) k stacked passes would over-extrude,
    // so the plain pass flow is used; flat bridges keep the pass flow with the bridge flow ratio applied.
    const Flow overhang_flow_pass = layer->object()->config().thick_bridges ?
        ext_flow_pass : this->flow(frPerimeter, h_pass).with_flow_ratio(region_config.bridge_flow_ratio);
    PerimeterGenerator::Parameters sub_params(
        h_pass,
        params.layer_id,
        this->flow(frPerimeter, h_pass),
        ext_flow_pass,
        this->flow(frFirstInternalPerimeter, h_pass),
        this->flow(frSecondInternalPerimeter, h_pass),
        overhang_flow_pass,
        this->flow(frSolidInfill, h_pass),
        sub_config,
        params.object_config,
        params.print_config,
        params.perimeter_regions,
        false);
    sub_params.two_pass_perimeters = false;

    // Per sub-plane inputs. This region is the only region of the layer, so its raw sub-slices are what a plain thin print would use as lslices.
    // The lower slices of sub-plane 0 are the topmost sub-slices of the lower layer and the upper slices of the last sub-plane are the first sub-slices
    // of the upper layer (the plane a thin print would see next to it) if the neighbouring layer has sub-slices and a single region,
    // otherwise the neighbouring layer's lslices. An empty upper plane means "topmost" (nullptr), as in a thin print.
    // (A literal reading of the spec - always the neighbouring lslices - differs from a thin print at top / bridge surfaces: top_one_perimeter_type and the overhang detection use them.)
    auto neighbour_sub_slices = [](const Layer *neighbour, bool topmost_plane) -> const ExPolygons* {
        if (neighbour == nullptr)
            return nullptr;
        if (neighbour->external_resolution() > 1 && neighbour->region_count() == 1) {
            const std::vector<ExPolygons> &subs = neighbour->get_region(0)->m_sub_slices;
            if (subs.size() == neighbour->external_resolution())
                return topmost_plane ? &subs.back() : &subs.front();
        }
        return &neighbour->lslices;
    };
    std::vector<const ExPolygons*> lower_slices(k), upper_slices(k);
    for (size_t j = 0; j < k; ++ j) {
        lower_slices[j] = j == 0 ? neighbour_sub_slices(layer->lower_layer, true) : &m_sub_slices[j - 1];
        upper_slices[j] = j + 1 == k ? neighbour_sub_slices(layer->upper_layer, false) : &m_sub_slices[j + 1];
        if (j + 1 != k || (layer->upper_layer != nullptr && layer->upper_layer->external_resolution() > 1))
            if (upper_slices[j] != nullptr && upper_slices[j]->empty())
                upper_slices[j] = nullptr;
    }
    // A fresh cache of the offsetted lower slices per sub-plane (shared by the islands of this region).
    std::vector<Polygons> lower_caches(k);

    // Bounding boxes of the sub-expolygons.
    std::vector<BoundingBoxes> sub_bboxes(k);
    for (size_t j = 0; j < k; ++ j) {
        sub_bboxes[j].reserve(m_sub_slices[j].size());
        for (const ExPolygon &ex : m_sub_slices[j])
            sub_bboxes[j].emplace_back(get_extents(ex));
    }
    // Claims of the sub-expolygons by the islands (an expolygon claimed by two islands makes both fall back).
    std::vector<std::vector<int>> sub_claims(k);
    std::vector<std::vector<int>> island_sub(slices.surfaces.size(), std::vector<int>(k, -1));
    std::vector<std::vector<int>> island_nsub(slices.surfaces.size(), std::vector<int>(k, 0));
    for (size_t j = 0; j < k; ++ j)
        sub_claims[j].assign(m_sub_slices[j].size(), 0);
    for (size_t s = 0; s < slices.surfaces.size(); ++ s) {
        const ExPolygon   &island = slices.surfaces[s].expolygon;
        const BoundingBox  island_bbox = get_extents(island);
        for (size_t j = 0; j < k; ++ j)
            for (size_t e = 0; e < m_sub_slices[j].size(); ++ e) {
                if (! island_bbox.overlap(sub_bboxes[j][e]))
                    continue;
                const double area_e = m_sub_slices[j][e].area();
                if (area_e <= 0.)
                    continue;
                // The sub-expolygon belongs to the island if at least half of it lies inside the island.
                double overlap = 0.;
                for (const ExPolygon &x : intersection_ex(ExPolygons{ m_sub_slices[j][e] }, ExPolygons{ island }))
                    overlap += x.area();
                if (overlap >= 0.5 * area_e) {
                    ++ island_nsub[s][j];
                    island_sub[s][j] = int(e);
                    ++ sub_claims[j][e];
                }
            }
    }

    std::vector<ExtrusionEntityCollection> passes(k - 1);
    // Debug dumps.
    std::vector<std::string> dump_hd(k);
    std::string              dump_thick;

    for (size_t s = 0; s < slices.surfaces.size(); ++ s) {
        const Surface        &surface = slices.surfaces[s];
        const ExtrusionRange &range   = perimeter_and_gapfill_ranges[range_offset + s].first;
        ++ stats.islands_total;
        if (range.empty() || range.size() != 1 || ! m_perimeters.entities[*range.begin()]->is_collection()) {
            ++ stats.fallback_no_externals;
            continue;
        }
        ExtrusionEntityCollection &island_coll = static_cast<ExtrusionEntityCollection&>(*m_perimeters.entities[*range.begin()]);
        // The external slots of the normal run.
        std::vector<size_t>                 slot_idx;
        std::vector<const ExtrusionEntity*> slot_entities;
        for (size_t i = 0; i < island_coll.entities.size(); ++ i)
            if (is_external_entity(island_coll.entities[i])) {
                slot_idx.emplace_back(i);
                slot_entities.emplace_back(island_coll.entities[i]);
            }
        if (slot_idx.empty()) {
            ++ stats.fallback_no_externals;
            continue;
        }
        // (1) Exactly one sub-expolygon per plane maps to this island and it is not shared with another island.
        bool ok = true;
        for (size_t j = 0; j < k && ok; ++ j)
            ok = island_nsub[s][j] == 1 && sub_claims[j][size_t(island_sub[s][j])] == 1;
        if (! ok) {
            ++ stats.fallback_mapping;
            BOOST_LOG_TRIVIAL(trace) << "External resolution fallback (mapping): layer " << layer->id() << " print_z " << layer->print_z << " island " << s;
            continue;
        }
        // (2) Equal hole counts.
        for (size_t j = 0; j < k && ok; ++ j)
            ok = m_sub_slices[j][size_t(island_sub[s][j])].holes.size() == surface.expolygon.holes.size();
        if (! ok) {
            ++ stats.fallback_holes;
            BOOST_LOG_TRIVIAL(trace) << "External resolution fallback (holes): layer " << layer->id() << " print_z " << layer->print_z << " island " << s;
            continue;
        }
        // The sub-runs.
        const ExternalSignature normal_sig = external_signature(slot_entities);
        std::vector<std::vector<ExtrusionEntity*>> ordered(k);
        std::vector<ExtrusionEntity*>              all_fresh;     // ownership until committed
        for (size_t j = 0; j < k && ok; ++ j) {
            Surface sub_surface(stInternal, m_sub_slices[j][size_t(island_sub[s][j])]);
            sub_surface.extra_perimeters = surface.extra_perimeters;
            ExtrusionEntityCollection tmp_loops, tmp_gaps;
            ExPolygons                tmp_fill;
            PerimeterGenerator::process_arachne(sub_params, sub_surface, lower_slices[j], upper_slices[j], lower_caches[j], tmp_loops, tmp_gaps, tmp_fill);
            ++ stats.sub_runs;
            std::vector<ExtrusionEntity*> fresh;
            if (! tmp_loops.entities.empty() && tmp_loops.entities.front()->is_collection()) {
                ExtrusionEntityCollection &isl = static_cast<ExtrusionEntityCollection&>(*tmp_loops.entities.front());
                for (ExtrusionEntity *&e : isl.entities)
                    if (is_external_entity(e)) {
                        fresh.emplace_back(e);
                        e = nullptr;        // ownership moved out of the scratch collection
                    }
            }
            all_fresh.insert(all_fresh.end(), fresh.begin(), fresh.end());
            // (3) Equal counts of external entities, closed loops and clockwise loops.
            std::vector<const ExtrusionEntity*> fresh_const(fresh.begin(), fresh.end());
            if (! (external_signature(fresh_const) == normal_sig) || ! match_externals_to_slots(slot_entities, fresh, ordered[j]))
                ok = false;
        }
        if (! ok) {
            for (ExtrusionEntity *e : all_fresh)
                delete e;
            ++ stats.fallback_counts;
            BOOST_LOG_TRIVIAL(trace) << "External resolution fallback (counts): layer " << layer->id() << " print_z " << layer->print_z << " island " << s;
            continue;
        }
        // Commit. Dump the normal (thick) externals first.
        if (ext_res_dump_dir() != nullptr)
            for (const ExtrusionEntity *e : slot_entities)
                ext_res_append_text(dump_thick, e);
        for (size_t j = 0; j + 1 < k; ++ j)
            for (ExtrusionEntity *e : ordered[j]) {
                if (ext_res_dump_dir() != nullptr)
                    ext_res_append_text(dump_hd[j], e);
                passes[j].entities.emplace_back(e);
            }
        for (size_t i = 0; i < slot_idx.size(); ++ i) {
            if (ext_res_dump_dir() != nullptr)
                ext_res_append_text(dump_hd[k - 1], ordered[k - 1][i]);
            delete island_coll.entities[slot_idx[i]];
            island_coll.entities[slot_idx[i]] = ordered[k - 1][i];
        }
        ++ stats.islands_processed;
    }

    bool any = false;
    for (const ExtrusionEntityCollection &c : passes)
        any |= ! c.empty();
    if (any)
        m_external_sub_passes = std::move(passes);

    if (const char *dir = ext_res_dump_dir(); dir != nullptr && any) {
        const double z0 = layer->slice_z - 0.5 * layer->height;
        for (size_t j = 0; j < k; ++ j) {
            ext_res_append_expolygons(dump_hd[j], m_sub_slices[j]);
            ext_res_write_file(ext_res_plane_name("plane", z0 + (double(j) + 0.5) * h_pass), dump_hd[j]);
        }
        ext_res_write_file(ext_res_plane_name("thick", layer->slice_z), dump_thick);
        // SVG: thick externals in grey, the passes in colours.
        static const char *colors[] = { "red", "blue", "green", "orange", "purple", "brown", "cyan", "magenta" };
        BoundingBox bbox;
        for (const ExPolygon &ex : layer->lslices)
            bbox.merge(get_extents(ex));
        if (bbox.defined) {
            char name[256];
            std::snprintf(name, sizeof(name), "%s/layer_%04zu_z%.3f.svg", dir, layer->id(), layer->print_z);
            ::Slic3r::SVG svg(name, bbox);
            const double w = scale_(0.04);
            for (const ExtrusionEntity *island : m_perimeters.entities)
                if (island->is_collection())
                    for (const ExtrusionEntity *e : static_cast<const ExtrusionEntityCollection*>(island)->entities)
                        if (is_external_entity(e)) {
                            Polylines pls;
                            e->collect_polylines(pls);
                            // The top pass (k-1) replaced the thick loops: drawn in the last colour.
                            svg.draw(pls, colors[(k - 1) % 8], w);
                        }
            for (size_t j = 0; j + 1 < k; ++ j) {
                Polylines pls;
                if (! m_external_sub_passes.empty())
                    m_external_sub_passes[j].collect_polylines(pls);
                svg.draw(pls, colors[j % 8], w);
            }
            svg.Close();
        }
    }
}

#if 1

// Extract surfaces of given type from surfaces, extract fill (layer) thickness of one of the surfaces.
static ExPolygons fill_surfaces_extract_expolygons(Surfaces &surfaces, std::initializer_list<SurfaceType> surface_types, double &thickness)
{
    size_t cnt = 0;
    for (const Surface &surface : surfaces)
        if (std::find(surface_types.begin(), surface_types.end(), surface.surface_type) != surface_types.end()) {
            ++cnt;
            thickness = surface.thickness;
        }
    if (cnt == 0)
        return {};

    ExPolygons out;
    out.reserve(cnt);
    for (Surface &surface : surfaces)
        if (std::find(surface_types.begin(), surface_types.end(), surface.surface_type) != surface_types.end())
            out.emplace_back(std::move(surface.expolygon));
    return out;
}

// Cache for detecting bridge orientation and merging regions with overlapping expansions.
struct Bridge {
    ExPolygon expolygon;
    uint32_t group_id;
    std::vector<Algorithm::RegionExpansionEx>::const_iterator bridge_expansion_begin;
};

// Group the bridge surfaces by overlaps.
uint32_t group_id(std::vector<Bridge> &bridges, uint32_t src_id) {
    uint32_t group_id = bridges[src_id].group_id;
    while (group_id != src_id) {
        src_id = group_id;
        group_id = bridges[src_id].group_id;
    }
    bridges[src_id].group_id = group_id;
    return group_id;
};

std::vector<Bridge> get_grouped_bridges(
    ExPolygons&& bridge_expolygons,
    const std::vector<Algorithm::RegionExpansionEx>& bridge_expansions
) {
    using namespace Algorithm;

    std::vector<Bridge> result;
    {
        result.reserve(bridge_expansions.size());
        uint32_t group_id = 0;
        using std::move_iterator;
        for (ExPolygon& expolygon : bridge_expolygons)
            result.push_back({ std::move(expolygon), group_id ++, bridge_expansions.end() });
    }


    // Detect overlaps of bridge anchors inside their respective shell regions.
    // bridge_expansions are sorted by boundary id and source id.
    for (auto expansion_iterator = bridge_expansions.begin(); expansion_iterator != bridge_expansions.end();) {
        auto boundary_region_begin = expansion_iterator;
        auto boundary_region_end = std::find_if(
            next(expansion_iterator),
            bridge_expansions.end(),
            [&](const RegionExpansionEx& expansion){
                return expansion.boundary_id != expansion_iterator->boundary_id;
            }
        );

        // Cache of bboxes per expansion boundary.
        std::vector<BoundingBox> bounding_boxes;
        bounding_boxes.reserve(std::distance(boundary_region_begin, boundary_region_end));
        std::transform(
            boundary_region_begin,
            boundary_region_end,
            std::back_inserter(bounding_boxes),
            [](const RegionExpansionEx& expansion){
                return get_extents(expansion.expolygon.contour);
            }
        );

        // For each bridge anchor of the current source:
        for (;expansion_iterator != boundary_region_end; ++expansion_iterator) {
            auto candidate_iterator = std::next(expansion_iterator);
            for (;candidate_iterator != boundary_region_end; ++candidate_iterator) {
                const BoundingBox& current_bounding_box{
                    bounding_boxes[expansion_iterator - boundary_region_begin]
                };
                const BoundingBox& candidate_bounding_box{
                    bounding_boxes[candidate_iterator - boundary_region_begin]
                };
                if (
                    expansion_iterator->src_id != candidate_iterator->src_id
                    && current_bounding_box.overlap(candidate_bounding_box)
                    // One may ignore holes, they are irrelevant for intersection test.
                    && !intersection(expansion_iterator->expolygon.contour, candidate_iterator->expolygon.contour).empty()
                ) {
                    // The two bridge regions intersect. Give them the same (lower) group id.
                    uint32_t id  = group_id(result, expansion_iterator->src_id);
                    uint32_t id2 = group_id(result, candidate_iterator->src_id);
                    if (id < id2)
                        result[id2].group_id = id;
                    else
                        result[id].group_id = id2;
                }
            }
        }
    }
    return result;
}

Surfaces merge_bridges(
    std::vector<Bridge>& bridges,
    const std::vector<Algorithm::RegionExpansionEx>& bridge_expansions,
    const float closing_radius
) {
    for (auto it = bridge_expansions.begin(); it != bridge_expansions.end(); ) {
        bridges[it->src_id].bridge_expansion_begin = it;
        uint32_t src_id = it->src_id;
        for (++ it; it != bridge_expansions.end() && it->src_id == src_id; ++ it) ;
    }

    Surfaces result;
    for (uint32_t bridge_id = 0; bridge_id < uint32_t(bridges.size()); ++ bridge_id) {
        if (group_id(bridges, bridge_id) == bridge_id) {
            // Head of the group.
            Polygons bridge_group;
            Polygons expansions;
            for (uint32_t bridge_id2 = bridge_id; bridge_id2 < uint32_t(bridges.size()); ++ bridge_id2) {
                if (group_id(bridges, bridge_id2) == bridge_id) {
                    append(bridge_group, to_polygons(std::move(bridges[bridge_id2].expolygon)));
                    auto it_bridge_expansion = bridges[bridge_id2].bridge_expansion_begin;
                    assert(it_bridge_expansion == bridge_expansions.end() || it_bridge_expansion->src_id == bridge_id2);
                    for (; it_bridge_expansion != bridge_expansions.end() && it_bridge_expansion->src_id == bridge_id2; ++ it_bridge_expansion)
                        append(expansions, to_polygons(it_bridge_expansion->expolygon));
                }
            }
            append(bridge_group, expansions);

            //NOTE: The current regularization of the shells can create small unasigned regions in the object (E.G. benchy)
            // without the following closing operation, those regions will stay unfilled and cause small holes in the expanded surface.
            // look for narrow_ensure_vertical_wall_thickness_region_radius filter.
            ExPolygons merged_bridges = closing_ex(bridge_group, closing_radius);
            // without safety offset, artifacts are generated (GH #2494)
            // union_safety_offset_ex(acc)

            for (ExPolygon &bridge_expolygon : merged_bridges) {
                const Lines lines{to_lines(diff_pl(to_polylines(bridge_expolygon), expand(expansions, float(SCALED_EPSILON))))};
                auto [bridging_dir, unsupported_dist] = detect_bridging_direction(lines, to_polygons(bridge_expolygon));
                Surface surface{ stBottomBridge, std::move(bridge_expolygon) };
                surface.bridge_angle = M_PI + std::atan2(bridging_dir.y(), bridging_dir.x());
                result.push_back(std::move(surface));
            }
        }
    }
    return result;
}

struct ExpansionResult {
    Algorithm::WaveSeeds anchors;
    std::vector<Algorithm::RegionExpansionEx> expansions;
};

ExpansionResult expand_expolygons(
    const ExPolygons& expolygons,
    std::vector<ExpansionZone>& expansion_zones
) {
    using namespace Algorithm;
    WaveSeeds bridge_anchors;
    std::vector<RegionExpansionEx> bridge_expansions;

    unsigned processed_bridges_count = 0;
    for (ExpansionZone& expansion_zone : expansion_zones) {
        WaveSeeds seeds{wave_seeds(
            expolygons,
            expansion_zone.expolygons,
            expansion_zone.parameters.tiny_expansion,
            true
        )};
        std::vector<RegionExpansionEx> expansions{propagate_waves_ex(
            seeds,
            expansion_zone.expolygons,
            expansion_zone.parameters
        )};

        for (WaveSeed &seed : seeds)
            seed.boundary += processed_bridges_count;
        for (RegionExpansionEx &expansion : expansions)
            expansion.boundary_id += processed_bridges_count;

        expansion_zone.expanded_into = ! expansions.empty();

        append(bridge_anchors, std::move(seeds));
        append(bridge_expansions, std::move(expansions));

        processed_bridges_count += expansion_zone.expolygons.size();
    }
    return {bridge_anchors, bridge_expansions};
}

// Extract bridging surfaces from "surfaces", expand them into "shells" using expansion_params,
// detect bridges.
// Trim "shells" by the expanded bridges.
Surfaces expand_bridges_detect_orientations(
    Surfaces &surfaces,
    std::vector<ExpansionZone>& expansion_zones,
    const float closing_radius
)
{
    using namespace Slic3r::Algorithm;

    double thickness;
    ExPolygons bridge_expolygons = fill_surfaces_extract_expolygons(surfaces, {stBottomBridge}, thickness);
    if (bridge_expolygons.empty())
        return {};

    // Calculate bridge anchors and their expansions in their respective shell region.
    ExpansionResult expansion_result{expand_expolygons(
        bridge_expolygons,
        expansion_zones
    )};

    std::vector<Bridge> bridges{get_grouped_bridges(
        std::move(bridge_expolygons),
        expansion_result.expansions
    )};
    bridge_expolygons.clear();

    std::sort(expansion_result.anchors.begin(), expansion_result.anchors.end(), Algorithm::lower_by_src_and_boundary);

    // Merge the groups with the same group id, produce surfaces by merging source overhangs with their newly expanded anchors.
    std::sort(expansion_result.expansions.begin(), expansion_result.expansions.end(), [](auto &l, auto &r) {
        return l.src_id < r.src_id || (l.src_id == r.src_id && l.boundary_id < r.boundary_id);
    });
    Surfaces out{merge_bridges(bridges, expansion_result.expansions, closing_radius)};

    // Clip by the expanded bridges.
    for (ExpansionZone& expansion_zone : expansion_zones)
        if (expansion_zone.expanded_into)
            expansion_zone.expolygons = diff_ex(expansion_zone.expolygons, out);
    return out;
}

Surfaces expand_merge_surfaces(
    Surfaces &surfaces,
    SurfaceType surface_type,
    std::vector<ExpansionZone>& expansion_zones,
    const float closing_radius,
    const double bridge_angle
)
{
    using namespace Slic3r::Algorithm;

    double thickness;
    ExPolygons src = fill_surfaces_extract_expolygons(surfaces, {surface_type}, thickness);
    if (src.empty())
        return {};

    unsigned processed_expolygons_count = 0;
    std::vector<RegionExpansion> expansions;
    for (ExpansionZone& expansion_zone : expansion_zones) {
        std::vector<RegionExpansion> zone_expansions = propagate_waves(src, expansion_zone.expolygons, expansion_zone.parameters);
        expansion_zone.expanded_into = !zone_expansions.empty();

        for (RegionExpansion &expansion : zone_expansions)
            expansion.boundary_id += processed_expolygons_count;

        processed_expolygons_count += expansion_zone.expolygons.size();
        append(expansions, std::move(zone_expansions));
    }

    std::vector<ExPolygon> expanded = merge_expansions_into_expolygons(std::move(src), std::move(expansions));
    //NOTE: The current regularization of the shells can create small unasigned regions in the object (E.G. benchy)
    // without the following closing operation, those regions will stay unfilled and cause small holes in the expanded surface.
    // look for narrow_ensure_vertical_wall_thickness_region_radius filter.
    expanded = closing_ex(expanded, closing_radius);
    // Trim the zones by the expanded expolygons.
    for (ExpansionZone& expansion_zone : expansion_zones)
        if (expansion_zone.expanded_into)
            expansion_zone.expolygons = diff_ex(expansion_zone.expolygons, expanded);

    Surface templ{ surface_type, {} };
    templ.bridge_angle = bridge_angle;
    Surfaces out;
    out.reserve(expanded.size());
    for (auto &expoly : expanded)
        out.emplace_back(templ, std::move(expoly));
    return out;
}

void LayerRegion::process_external_surfaces(const Layer *lower_layer, const Polygons *lower_layer_covered)
{
    using namespace Slic3r::Algorithm;

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_fill_surfaces_to_svg_debug("4_process_external_surfaces-initial");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */

    // Width of the perimeters.
    float shell_width = 0;
    float expansion_min = 0;
    if (int num_perimeters = this->region().config().perimeters; num_perimeters > 0) {
        Flow external_perimeter_flow = this->flow(frExternalPerimeter);
        Flow first_internal_perimeter_flow = this->flow(frFirstInternalPerimeter);
        Flow second_internal_perimeter_flow = this->flow(frSecondInternalPerimeter);
        Flow perimeter_flow          = this->flow(frPerimeter);
        shell_width = 0.5f * external_perimeter_flow.scaled_width() + external_perimeter_flow.scaled_spacing();
        if (num_perimeters >= 2)
            shell_width += first_internal_perimeter_flow.scaled_spacing();
        if (num_perimeters >= 3)
            shell_width += second_internal_perimeter_flow.scaled_spacing();
        if (num_perimeters > 3)
            shell_width += perimeter_flow.scaled_spacing() * (num_perimeters - 3);
        expansion_min = perimeter_flow.scaled_spacing();
    } else {
        // TODO: Maybe there is better solution when printing with zero perimeters, but this works reasonably well, given the situation
        shell_width   = float(SCALED_EPSILON);
        expansion_min = float(SCALED_EPSILON);;
    }

    // Scaled expansions of the respective external surfaces.
    float                           expansion_top           = shell_width * sqrt(2.);
    float                           expansion_bottom        = expansion_top;
    float                           expansion_bottom_bridge = expansion_top;
    // Expand by waves of expansion_step size (expansion_step is scaled), but with no more steps than max_nr_expansion_steps.
    static constexpr const float    expansion_step          = scaled<float>(0.1);
    // Don't take more than max_nr_steps for small expansion_step.
    static constexpr const size_t   max_nr_expansion_steps  = 5;
    // Radius (with added epsilon) to absorb empty regions emering from regularization of ensuring, viz  const float narrow_ensure_vertical_wall_thickness_region_radius = 0.5f * 0.65f * min_perimeter_infill_spacing;
    const float closing_radius = 0.55f * 0.65f * 1.05f * this->flow(frSolidInfill).scaled_spacing();

    // Expand the top / bottom / bridge surfaces into the shell thickness solid infills.
    double     layer_thickness;
    ExPolygons shells = union_ex(fill_surfaces_extract_expolygons(m_fill_surfaces.surfaces, { stInternalSolid }, layer_thickness));
    ExPolygons sparse = union_ex(fill_surfaces_extract_expolygons(m_fill_surfaces.surfaces, { stInternal }, layer_thickness));
    ExPolygons top_expolygons = union_ex(fill_surfaces_extract_expolygons(m_fill_surfaces.surfaces, { stTop }, layer_thickness));
    const auto expansion_params_into_sparse_infill = RegionExpansionParameters::build(expansion_min, expansion_step, max_nr_expansion_steps);
    const auto expansion_params_into_solid_infill  = RegionExpansionParameters::build(expansion_bottom_bridge, expansion_step, max_nr_expansion_steps);

    std::vector<ExpansionZone> expansion_zones{
        ExpansionZone{std::move(shells), expansion_params_into_solid_infill},
        ExpansionZone{std::move(sparse), expansion_params_into_sparse_infill},
        ExpansionZone{std::move(top_expolygons), expansion_params_into_solid_infill},
    };

    SurfaceCollection bridges;
    {
        BOOST_LOG_TRIVIAL(trace) << "Processing external surface, detecting bridges. layer" << this->layer()->print_z;
        const double custom_angle = this->region().config().bridge_angle.value;
        bridges.surfaces = custom_angle > 0 ?
            expand_merge_surfaces(m_fill_surfaces.surfaces, stBottomBridge, expansion_zones, closing_radius, Geometry::deg2rad(custom_angle)) :
            expand_bridges_detect_orientations(m_fill_surfaces.surfaces, expansion_zones, closing_radius);
        BOOST_LOG_TRIVIAL(trace) << "Processing external surface, detecting bridges - done";
#if 0
        {
            static int iRun = 0;
            bridges.export_to_svg(debug_out_path("bridges-after-grouping-%d.svg", iRun++), true);
        }
#endif
    }

    m_fill_surfaces.remove_types({ stTop });
    {
        Surface top_templ(stTop, {});
        top_templ.thickness = layer_thickness;
        m_fill_surfaces.append(std::move(expansion_zones.back().expolygons), top_templ);
    }

    expansion_zones.pop_back();

    expansion_zones.at(0).parameters = RegionExpansionParameters::build(expansion_bottom, expansion_step, max_nr_expansion_steps);
    Surfaces bottoms = expand_merge_surfaces(m_fill_surfaces.surfaces, stBottom, expansion_zones, closing_radius);

    expansion_zones.at(0).parameters = RegionExpansionParameters::build(expansion_top, expansion_step, max_nr_expansion_steps);
    Surfaces tops = expand_merge_surfaces(m_fill_surfaces.surfaces, stTop, expansion_zones, closing_radius);

//    m_fill_surfaces.remove_types({ stBottomBridge, stBottom, stTop, stInternal, stInternalSolid });
    m_fill_surfaces.clear();
    unsigned zones_expolygons_count = 0;
    for (const ExpansionZone& zone : expansion_zones)
        zones_expolygons_count += zone.expolygons.size();
    reserve_more(m_fill_surfaces.surfaces, zones_expolygons_count + bridges.size() + bottoms.size() + tops.size());
    {
        Surface solid_templ(stInternalSolid, {});
        solid_templ.thickness = layer_thickness;
        m_fill_surfaces.append(std::move(expansion_zones[0].expolygons), solid_templ);
    }
    {
        Surface sparse_templ(stInternal, {});
        sparse_templ.thickness = layer_thickness;
        m_fill_surfaces.append(std::move(expansion_zones[1].expolygons), sparse_templ);
    }
    m_fill_surfaces.append(std::move(bridges.surfaces));
    m_fill_surfaces.append(std::move(bottoms));
    m_fill_surfaces.append(std::move(tops));

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_fill_surfaces_to_svg_debug("4_process_external_surfaces-final");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */
}
#else

//#define EXTERNAL_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 3.
//#define EXTERNAL_SURFACES_OFFSET_PARAMETERS ClipperLib::jtMiter, 1.5
#define EXTERNAL_SURFACES_OFFSET_PARAMETERS ClipperLib::jtSquare, 0.

void LayerRegion::process_external_surfaces(const Layer *lower_layer, const Polygons *lower_layer_covered)
{
    const bool      has_infill = this->region().config().fill_density.value > 0.;
//    const float		margin     = scaled<float>(0.1); // float(scale_(EXTERNAL_INFILL_MARGIN));
    const float     margin     = float(scale_(EXTERNAL_INFILL_MARGIN));

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_fill_surfaces_to_svg_debug("4_process_external_surfaces-initial");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */

    // 1) Collect bottom and bridge surfaces, each of them grown by a fixed 3mm offset
    // for better anchoring.
    // Bottom surfaces, grown.
    Surfaces                    bottom;
    // Bridge surfaces, initialy not grown.
    Surfaces                    bridges;
    // Top surfaces, grown.
    Surfaces                    top;
    // Internal surfaces, not grown.
    Surfaces                    internal;
    // Areas, where an infill of various types (top, bottom, bottom bride, sparse, void) could be placed.
    Polygons                    fill_boundaries = to_polygons(this->fill_expolygons());

    // Collect top surfaces and internal surfaces.
    // Collect fill_boundaries: If we're slicing with no infill, we can't extend external surfaces over non-existent infill.
    // This loop destroys the surfaces (aliasing this->fill_surfaces.surfaces) by moving into top/internal/fill_boundaries!

    {
        // Voids are sparse infills if infill rate is zero.
        Polygons voids;
        for (const Surface &surface : this->fill_surfaces()) {
            assert(! surface.empty());
            if (! surface.empty()) {
                if (surface.is_top()) {
                    // Collect the top surfaces, inflate them and trim them by the bottom surfaces.
                    // This gives the priority to bottom surfaces.
                    surfaces_append(top, offset_ex(surface.expolygon, margin, EXTERNAL_SURFACES_OFFSET_PARAMETERS), surface);
                } else if (surface.surface_type == stBottom || (surface.surface_type == stBottomBridge && lower_layer == nullptr)) {
                    // Grown by 3mm.
                    surfaces_append(bottom, offset_ex(surface.expolygon, margin, EXTERNAL_SURFACES_OFFSET_PARAMETERS), surface);
                } else if (surface.surface_type == stBottomBridge) {
                    bridges.emplace_back(surface);
                } else {
                    assert(surface.is_internal());
                	assert(surface.surface_type == stInternal || surface.surface_type == stInternalSolid);
                	if (! has_infill && lower_layer != nullptr)
                		polygons_append(voids, surface.expolygon);
                	internal.emplace_back(std::move(surface));
                }
            }
        }
        if (! voids.empty()) {
            // There are some voids (empty infill regions) on this layer. Usually one does not want to expand
            // any infill into these voids, with the exception the expanded infills are supported by layers below
            // with nonzero inill.
            assert(! has_infill && lower_layer != nullptr);
        	// Remove voids from fill_boundaries, that are not supported by the layer below.
            Polygons lower_layer_covered_tmp;
            if (lower_layer_covered == nullptr) {
            	lower_layer_covered = &lower_layer_covered_tmp;
            	lower_layer_covered_tmp = to_polygons(lower_layer->lslices);
            }
            if (! lower_layer_covered->empty())
                // Allow the top / bottom surfaces to expand into the voids of this layer if supported by the layer below.
            	voids = diff(voids, *lower_layer_covered);
            if (! voids.empty())
                fill_boundaries = diff(fill_boundaries, voids);
        }
    }

#if 0
    {
        static int iRun = 0;
        bridges.export_to_svg(debug_out_path("bridges-before-grouping-%d.svg", iRun ++), true);
    }
#endif

    if (bridges.empty())
    {
        fill_boundaries = union_safety_offset(fill_boundaries);
    } else
    {
        // 1) Calculate the inflated bridge regions, each constrained to its island.
        ExPolygons               fill_boundaries_ex = union_safety_offset_ex(fill_boundaries);
        std::vector<Polygons>    bridges_grown;
        std::vector<BoundingBox> bridge_bboxes;

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
        {
            static int iRun = 0;
            SVG svg(debug_out_path("4_process_external_surfaces-fill_regions-%d.svg", iRun ++).c_str(), get_extents(fill_boundaries_ex));
            svg.draw(fill_boundaries_ex);
            svg.draw_outline(fill_boundaries_ex, "black", "blue", scale_(0.05)); 
            svg.Close();
        }
//        export_region_fill_surfaces_to_svg_debug("4_process_external_surfaces-initial");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */
 
        {
            // Bridge expolygons, grown, to be tested for intersection with other bridge regions.
            std::vector<BoundingBox> fill_boundaries_ex_bboxes = get_extents_vector(fill_boundaries_ex);
            bridges_grown.reserve(bridges.size());
            bridge_bboxes.reserve(bridges.size());
            for (size_t i = 0; i < bridges.size(); ++ i) {
                // Find the island of this bridge.
                const Point pt = bridges[i].expolygon.contour.points.front();
                int idx_island = -1;
                for (int j = 0; j < int(fill_boundaries_ex.size()); ++ j)
                    if (fill_boundaries_ex_bboxes[j].contains(pt) && 
                        fill_boundaries_ex[j].contains(pt)) {
                        idx_island = j;
                        break;
                    }
                // Grown by 3mm.
                Polygons polys = offset(bridges[i].expolygon, margin, EXTERNAL_SURFACES_OFFSET_PARAMETERS);
                if (idx_island == -1) {
				    BOOST_LOG_TRIVIAL(trace) << "Bridge did not fall into the source region!";
                } else {
                    // Found an island, to which this bridge region belongs. Trim the expanded bridging region
                    // with its source region, so it does not overflow into a neighbor region.
                    polys = intersection(polys, fill_boundaries_ex[idx_island]);
                }
                bridge_bboxes.push_back(get_extents(polys));
                bridges_grown.push_back(std::move(polys));
            }
        }

        // 2) Group the bridge surfaces by overlaps.
        std::vector<size_t> bridge_group(bridges.size(), (size_t)-1);
        size_t n_groups = 0; 
        for (size_t i = 0; i < bridges.size(); ++ i) {
            // A grup id for this bridge.
            size_t group_id = (bridge_group[i] == size_t(-1)) ? (n_groups ++) : bridge_group[i];
            bridge_group[i] = group_id;
            // For all possibly overlaping bridges:
            for (size_t j = i + 1; j < bridges.size(); ++ j) {
                if (! bridge_bboxes[i].overlap(bridge_bboxes[j]))
                    continue;
                if (intersection(bridges_grown[i], bridges_grown[j]).empty())
                    continue;
                // The two bridge regions intersect. Give them the same group id.
                if (bridge_group[j] != size_t(-1)) {
                    // The j'th bridge has been merged with some other bridge before.
                    size_t group_id_new = bridge_group[j];
                    for (size_t k = 0; k < j; ++ k)
                        if (bridge_group[k] == group_id)
                            bridge_group[k] = group_id_new;
                    group_id = group_id_new;
                }
                bridge_group[j] = group_id;
            }
        }

        // 3) Merge the groups with the same group id, detect bridges.
        {
			BOOST_LOG_TRIVIAL(trace) << "Processing external surface, detecting bridges. layer" << this->layer()->print_z << ", bridge groups: " << n_groups;
            for (size_t group_id = 0; group_id < n_groups; ++ group_id) {
                size_t n_bridges_merged = 0;
                size_t idx_last = (size_t)-1;
                for (size_t i = 0; i < bridges.size(); ++ i) {
                    if (bridge_group[i] == group_id) {
                        ++ n_bridges_merged;
                        idx_last = i;
                    }
                }
                if (n_bridges_merged == 0)
                    // This group has no regions assigned as these were moved into another group.
                    continue;
                // Collect the initial ungrown regions and the grown polygons.
                ExPolygons  initial;
                Polygons    grown;
                for (size_t i = 0; i < bridges.size(); ++ i) {
                    if (bridge_group[i] != group_id)
                        continue;
                    initial.push_back(std::move(bridges[i].expolygon));
                    polygons_append(grown, bridges_grown[i]);
                }
                // detect bridge direction before merging grown surfaces otherwise adjacent bridges
                // would get merged into a single one while they need different directions
                // also, supply the original expolygon instead of the grown one, because in case
                // of very thin (but still working) anchors, the grown expolygon would go beyond them
                double custom_angle = Geometry::deg2rad(this->region().config().bridge_angle.value);
                if (custom_angle > 0.0) {
                    bridges[idx_last].bridge_angle = custom_angle;
                } else {
                    auto [bridging_dir, unsupported_dist] = detect_bridging_direction(to_polygons(initial), to_polygons(lower_layer->lslices));
                    bridges[idx_last].bridge_angle = PI + std::atan2(bridging_dir.y(), bridging_dir.x());

                    // #if 1
                    //     coordf_t    stroke_width = scale_(0.06);
                    //     BoundingBox bbox         = get_extents(initial);
                    //     bbox.offset(scale_(1.));
                    //     ::Slic3r::SVG
                    //     svg(debug_out_path(("bridge"+std::to_string(bridges[idx_last].bridge_angle)+"_"+std::to_string(this->layer()->bottom_z())).c_str()),
                    //     bbox);

                    //     svg.draw(initial, "cyan");
                    //     svg.draw(to_lines(lower_layer->lslices), "green", stroke_width);
                    // #endif
                }

                /*
                BridgeDetector bd(initial, lower_layer->lslices, this->bridging_flow(frInfill).scaled_width());
                #ifdef SLIC3R_DEBUG
                printf("Processing bridge at layer %zu:\n", this->layer()->id());
                #endif
				double custom_angle = Geometry::deg2rad(this->region().config().bridge_angle.value);
				if (bd.detect_angle(custom_angle)) {
                    bridges[idx_last].bridge_angle = bd.angle;
                    if (this->layer()->object()->has_support()) {
//                        polygons_append(this->bridged, bd.coverage());
                        append(m_unsupported_bridge_edges, bd.unsupported_edges());
                    }
				} else if (custom_angle > 0) {
					// Bridge was not detected (likely it is only supported at one side). Still it is a surface filled in
					// using a bridging flow, therefore it makes sense to respect the custom bridging direction.
					bridges[idx_last].bridge_angle = custom_angle;
				}
                */
                // without safety offset, artifacts are generated (GH #2494)
                surfaces_append(bottom, union_safety_offset_ex(grown), bridges[idx_last]);
            }

            fill_boundaries = to_polygons(fill_boundaries_ex);
			BOOST_LOG_TRIVIAL(trace) << "Processing external surface, detecting bridges - done";
		}

    #if 0
        {
            static int iRun = 0;
            bridges.export_to_svg(debug_out_path("bridges-after-grouping-%d.svg", iRun ++), true);
        }
    #endif
    }

    Surfaces new_surfaces;
    {
        // Intersect the grown surfaces with the actual fill boundaries.
        Polygons bottom_polygons = to_polygons(bottom);
        // Merge top and bottom in a single collection.
        surfaces_append(top, std::move(bottom));
        for (size_t i = 0; i < top.size(); ++ i) {
            Surface &s1 = top[i];
            if (s1.empty())
                continue;
            Polygons polys;
            polygons_append(polys, to_polygons(std::move(s1)));
            for (size_t j = i + 1; j < top.size(); ++ j) {
                Surface &s2 = top[j];
                if (! s2.empty() && surfaces_could_merge(s1, s2)) {
                    polygons_append(polys, to_polygons(std::move(s2)));
                    s2.clear();
                }
            }
            if (s1.is_top())
                // Trim the top surfaces by the bottom surfaces. This gives the priority to the bottom surfaces.
                polys = diff(polys, bottom_polygons);
            surfaces_append(
                new_surfaces,
                // Don't use a safety offset as fill_boundaries were already united using the safety offset.
                intersection_ex(polys, fill_boundaries),
                s1);
        }
    }
    
    // Subtract the new top surfaces from the other non-top surfaces and re-add them.
    Polygons new_polygons = to_polygons(new_surfaces);
    for (size_t i = 0; i < internal.size(); ++ i) {
        Surface &s1 = internal[i];
        if (s1.empty())
            continue;
        Polygons polys;
        polygons_append(polys, to_polygons(std::move(s1)));
        for (size_t j = i + 1; j < internal.size(); ++ j) {
            Surface &s2 = internal[j];
            if (! s2.empty() && surfaces_could_merge(s1, s2)) {
                polygons_append(polys, to_polygons(std::move(s2)));
                s2.clear();
            }
        }
        ExPolygons new_expolys = diff_ex(polys, new_polygons);
        polygons_append(new_polygons, to_polygons(new_expolys));
        surfaces_append(new_surfaces, std::move(new_expolys), s1);
    }
    
    m_fill_surfaces.surfaces = std::move(new_surfaces);

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_fill_surfaces_to_svg_debug("4_process_external_surfaces-final");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */
}
#endif

void LayerRegion::prepare_fill_surfaces()
{
#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_slices_to_svg_debug("2_prepare_fill_surfaces-initial");
    export_region_fill_surfaces_to_svg_debug("2_prepare_fill_surfaces-initial");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */ 

    /*  Note: in order to make the psPrepareInfill step idempotent, we should never
        alter fill_surfaces boundaries on which our idempotency relies since that's
        the only meaningful information returned by psPerimeters. */
    
    bool spiral_vase = this->layer()->object()->print()->config().spiral_vase;

    // if no solid layers are requested, turn top/bottom surfaces to internal
    // For Lightning infill, infill_only_where_needed is ignored because both
    // do a similar thing, and their combination doesn't make much sense.
    if (! spiral_vase && this->region().config().top_solid_layers == 0) {
        for (Surface &surface : m_fill_surfaces)
            if (surface.is_top())
                surface.surface_type = /*this->layer()->object()->config().infill_only_where_needed && this->region().config().fill_pattern != ipLightning ? stInternalVoid :*/ stInternal;
    }
    if (this->region().config().bottom_solid_layers == 0) {
        for (Surface &surface : m_fill_surfaces)
            if (surface.is_bottom()) // (surface.surface_type == stBottom)
                surface.surface_type = stInternal;
    }

    // turn too small internal regions into solid regions according to the user setting
    if (! spiral_vase && this->region().config().fill_density.value > 0) {
        // scaling an area requires two calls!
        double min_area = scale_(scale_(this->region().config().solid_infill_below_area.value));
        for (Surface &surface : m_fill_surfaces)
            if (surface.surface_type == stInternal && surface.area() <= min_area)
                surface.surface_type = stInternalSolid;
    }

#ifdef SLIC3R_DEBUG_SLICE_PROCESSING
    export_region_slices_to_svg_debug("2_prepare_fill_surfaces-final");
    export_region_fill_surfaces_to_svg_debug("2_prepare_fill_surfaces-final");
#endif /* SLIC3R_DEBUG_SLICE_PROCESSING */
}

double LayerRegion::infill_area_threshold() const
{
    double ss = this->flow(frSolidInfill).scaled_spacing();
    return ss*ss;
}

void LayerRegion::trim_surfaces(const Polygons &trimming_polygons)
{
#ifndef NDEBUG
    for (const Surface &surface : this->slices())
        assert(surface.surface_type == stInternal);
#endif /* NDEBUG */
	m_slices.set(intersection_ex(this->slices().surfaces, trimming_polygons), stInternal);
}

void LayerRegion::elephant_foot_compensation_step(const float elephant_foot_compensation_perimeter_step, const Polygons &trimming_polygons)
{
#ifndef NDEBUG
    for (const Surface &surface : this->slices())
        assert(surface.surface_type == stInternal);
#endif /* NDEBUG */
    Polygons tmp = intersection(this->slices().surfaces, trimming_polygons);
    append(tmp, diff(this->slices().surfaces, opening(this->slices().surfaces, elephant_foot_compensation_perimeter_step)));
    m_slices.set(union_ex(tmp), stInternal);
}

void LayerRegion::export_region_slices_to_svg(const char *path) const
{
    BoundingBox bbox;
    for (const Surface &surface : this->slices())
        bbox.merge(get_extents(surface.expolygon));
    Point legend_size = export_surface_type_legend_to_svg_box_size();
    Point legend_pos(bbox.min(0), bbox.max(1));
    bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));

    SVG svg(path, bbox);
    const float transparency = 0.5f;
    for (const Surface &surface : this->slices())
        svg.draw(surface.expolygon, surface_type_to_color_name(surface.surface_type), transparency);
    for (const Surface &surface : this->fill_surfaces())
        svg.draw(surface.expolygon.lines(), surface_type_to_color_name(surface.surface_type));
    export_surface_type_legend_to_svg(svg, legend_pos);
    svg.Close();
}

// Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
void LayerRegion::export_region_slices_to_svg_debug(const char *name) const
{
    static std::map<std::string, size_t> idx_map;
    size_t &idx = idx_map[name];
    this->export_region_slices_to_svg(debug_out_path("LayerRegion-slices-%s-%d.svg", name, idx ++).c_str());
}

void LayerRegion::export_region_fill_surfaces_to_svg(const char *path) const
{
    BoundingBox bbox;
    for (const Surface &surface : this->fill_surfaces())
        bbox.merge(get_extents(surface.expolygon));
    Point legend_size = export_surface_type_legend_to_svg_box_size();
    Point legend_pos(bbox.min(0), bbox.max(1));
    bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));

    SVG svg(path, bbox);
    const float transparency = 0.5f;
    for (const Surface &surface : this->fill_surfaces()) {
        svg.draw(surface.expolygon, surface_type_to_color_name(surface.surface_type), transparency);
        svg.draw_outline(surface.expolygon, "black", "blue", scale_(0.05)); 
    }
    export_surface_type_legend_to_svg(svg, legend_pos);
    svg.Close();
}

// Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
void LayerRegion::export_region_fill_surfaces_to_svg_debug(const char *name) const
{
    static std::map<std::string, size_t> idx_map;
    size_t &idx = idx_map[name];
    this->export_region_fill_surfaces_to_svg(debug_out_path("LayerRegion-fill_surfaces-%s-%d.svg", name, idx ++).c_str());
}

}
 

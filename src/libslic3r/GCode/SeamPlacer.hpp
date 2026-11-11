///|/ Copyright (c) Prusa Research 2020 - 2022 Pavel Mikuš @Godrak, Lukáš Matěna @lukasmatena,
/// Vojtěch Bubník @bubnikv
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/
#ifndef libslic3r_SeamPlacer_hpp_
#define libslic3r_SeamPlacer_hpp_

#include <map>
#include <optional>
#include <vector>
#include <memory>
#include <atomic>

#include "libslic3r/GCode/SeamAligned.hpp"
#include "libslic3r/GCode/SeamScarf.hpp"
#include "libslic3r/Polygon.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/GCode/SeamPerimeters.hpp"
#include "libslic3r/GCode/SeamChoice.hpp"
#include "libslic3r/GCode/ModelVisibility.hpp"

namespace Slic3r::Seams {

using ObjectSeams =
    std::unordered_map<const PrintObject *, std::vector<std::vector<SeamPerimeterChoice>>>;
using ObjectLayerPerimeters = std::unordered_map<const PrintObject *, Perimeters::LayerPerimeters>;

struct Params
{
    double max_nearest_detour;
    double rear_tolerance;
    double rear_y_offset;
    Aligned::Params aligned;
    double max_distance{};
    unsigned random_seed{};
    double convex_visibility_modifier{};
    double concave_visibility_modifier{};
    Perimeters::PerimeterParams perimeter;
    Slic3r::ModelInfo::Visibility::Params visibility;
};

std::ostream& operator<<(std::ostream& os, const Params& params);

// Walks `arc` (unscaled mm, may be negative or exceed the loop length; wraps) along the closed chain
// `lines` (consecutive, in order) from `start_point` lying on lines[start_line]. Returns the line index
// and point reached. Used by the half-loop inner seam stagger.
std::pair<std::size_t, Vec2d> point_at_arc_length(
    const Linesf &lines, std::size_t start_line, const Vec2d &start_point, double arc
);

class Placer
{
public:
    static Params get_params(const DynamicPrintConfig &config);

    void init(
        SpanOfConstPtrs<PrintObject> objects,
        const Params &params,
        const std::function<void(void)> &throw_if_canceled
    );

    boost::variant<Point, Scarf::Scarf> place_seam(
        const Layer *layer, const PrintRegion *region, const ExtrusionLoop &loop, const bool flipped, const Point &last_pos,
        // Rotating loop mode: virtual pass index of an external sub-pass.
        std::optional<std::size_t> sub_pass = std::nullopt
    ) const;

private:
    Params params;
    ObjectSeams seams_per_object;
    // Rotating loop mode with external resolution: [layer][pass][perimeter] seam choices, parallel to
    // seams_per_object[layer] (the perimeter geometry is identical for all passes).
    std::map<const PrintObject *, std::vector<std::vector<std::vector<SeamChoice>>>> rotating_pass_seams;
    ObjectLayerPerimeters perimeters_per_layer;
};

} // namespace Slic3r::Seams

#endif // libslic3r_SeamPlacer_hpp_

// External perimeter resolution (spike part A): option, per-layer factor, sub-slices, high-definition external runs.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <cmath>
#include <memory>
#include <set>
#include <sstream>
#include <string>
#include <vector>

#include "libslic3r/Config.hpp"
#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/LayerRegion.hpp"
#include "libslic3r/Line.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_data.hpp"

using namespace Slic3r;
using Catch::Approx;

TEST_CASE("External resolution factor table", "[ExternalResolution]")
{
    CHECK(external_resolution_factor(0.10, 0.07, 4) == 1);
    CHECK(external_resolution_factor(0.14, 0.07, 4) == 2);
    CHECK(external_resolution_factor(0.15, 0.07, 4) == 2);
    CHECK(external_resolution_factor(0.20, 0.07, 4) == 2);
    // 0.21 / 0.07 = 2.9999999999999996: floor(h / min_lh) would give 2.
    CHECK(external_resolution_factor(0.21, 0.07, 4) == 3);
    CHECK(external_resolution_factor(0.28, 0.07, 4) == 4);
    CHECK(external_resolution_factor(0.30, 0.07, 4) == 4);
    CHECK(external_resolution_factor(0.35, 0.07, 4) == 4);
    CHECK(external_resolution_factor(0.30, 0.07, 2) == 2);
    CHECK(external_resolution_factor(0.30, 0.07, 1) == 1);
    // min_layer_height 0 means the 0.07 default.
    CHECK(external_resolution_factor(0.20, 0., 4) == 2);
    CHECK(external_resolution_factor(0.21, 0., 4) == 3);
    // Floor of 0.01.
    CHECK(external_resolution_factor(0.04, 0.001, 8) == 4);
}

TEST_CASE("External resolution option parses, serializes, defaults to 1 and validates", "[ExternalResolution]")
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
    CHECK(cfg.opt_int("external_perimeter_resolution") == 1);
    cfg.set_deserialize_strict("external_perimeter_resolution", "3");
    CHECK(cfg.opt_int("external_perimeter_resolution") == 3);
    CHECK(cfg.opt_serialize("external_perimeter_resolution") == "3");
    const ConfigOptionDef *def = print_config_def.get("external_perimeter_resolution");
    REQUIRE(def != nullptr);
    CHECK(def->min == 1);
    CHECK(def->max == 8);
    CHECK(def->mode == comExpert);

    auto validate = [](std::initializer_list<ConfigBase::SetDeserializeItem> items) {
        Print print;
        Model model;
        Test::init_print({ Test::TestMesh::cube_20x20x20 }, print, model, items);
        return print.validate();
    };
    CHECK(validate({ { "external_perimeter_resolution", "2" }, { "perimeter_generator", "arachne" } }).empty());
    CHECK(! validate({ { "external_perimeter_resolution", "2" }, { "perimeter_generator", "classic" } }).empty());
    CHECK(validate({ { "external_perimeter_resolution", "1" }, { "perimeter_generator", "classic" } }).empty());
    CHECK(! validate({ { "external_perimeter_resolution", "2" }, { "perimeter_generator", "arachne" }, { "spiral_vase", "1" } }).empty());
}

namespace {

struct ExtSliced {
    Model model;
    Print print;
};

std::unique_ptr<ExtSliced> ext_slice(const TriangleMesh &mesh, double layer_height, double first_layer_height, int resolution)
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config_with({
        { "skirts",                       0 },
        { "perimeters",                   3 },
        { "layer_height",                 layer_height },
        { "first_layer_height",           first_layer_height },
        { "perimeter_generator",          "arachne" },
        { "external_perimeter_resolution", resolution },
        { "fill_density",                 "5%" },
        { "top_one_perimeter_type",       "none" },
        { "cooling",                      "0" }
    });
    auto s = std::make_unique<ExtSliced>();
    Test::init_print(std::vector<TriangleMesh>{ mesh }, s->print, s->model, cfg);
    s->print.set_status_silent();
    s->print.process();
    return s;
}

double min_distance_to_loop(const Point &pt, const Points &loop)
{
    double d = std::numeric_limits<double>::max();
    for (size_t i = 0; i + 1 < loop.size(); ++ i)
        d = std::min(d, line_alg::distance_to(Line(loop[i], loop[i + 1]), pt));
    return d;
}

// External loops (perimeter_index 0, closed) as point lists, of the thick perimeters or of a sub-pass collection.
void collect_loops(const ExtrusionEntityCollection &coll, std::vector<Points> &out)
{
    for (const ExtrusionEntity *e : coll.entities) {
        if (e->is_collection())
            collect_loops(*static_cast<const ExtrusionEntityCollection*>(e), out);
        else if (auto *loop = dynamic_cast<const ExtrusionLoop*>(e)) {
            if (! loop->paths.empty() && loop->paths.front().attributes().perimeter_index == std::optional<uint16_t>(0)) {
                Points pts;
                loop->collect_points(pts);
                out.emplace_back(std::move(pts));
            }
        }
    }
}

double hausdorff(const Points &a, const Points &b)
{
    double d = 0.;
    for (const Point &p : a) d = std::max(d, min_distance_to_loop(p, b));
    for (const Point &p : b) d = std::max(d, min_distance_to_loop(p, a));
    return unscale<double>(d);
}

} // namespace

TEST_CASE("External resolution sub-slices of a cone match r(z) and the pass externals match a thin print", "[ExternalResolution]")
{
    const double R = 10., H = 20.;
    TriangleMesh cone = make_cone(R, H);
    auto thick = ext_slice(cone, 0.3, 0.3, 3);
    auto plain = ext_slice(cone, 0.3, 0.3, 1);
    auto thin  = ext_slice(cone, 0.1, 0.3, 1);
    const PrintObject &obj_thick = *thick->print.objects().front();
    const PrintObject &obj_plain = *plain->print.objects().front();
    const PrintObject &obj_thin  = *thin->print.objects().front();
    REQUIRE(obj_thick.layer_count() == obj_plain.layer_count());

    // k == 1: nothing allocated, normal slices identical.
    for (size_t i = 0; i < obj_plain.layer_count(); ++ i) {
        const Layer *lp = obj_plain.get_layer(int(i));
        CHECK(lp->external_resolution() == 1);
        CHECK(lp->get_region(0)->sub_slices().empty());
        CHECK(lp->get_region(0)->external_sub_passes().empty());
        const Layer *lt = obj_thick.get_layer(int(i));
        CHECK(lt->lslices.size() == lp->lslices.size());
        CHECK(std::abs(lt->lslices.front().area() - lp->lslices.front().area()) == Approx(0.).margin(1.));
    }

    // Sub-slice areas: the cone radius at the sub-plane within 2 micron.
    size_t checked = 0, loops_checked = 0;
    for (size_t i = 0; i < obj_thick.layer_count(); ++ i) {
        const Layer *layer = obj_thick.get_layer(int(i));
        if (i == 0) {
            CHECK(layer->external_resolution() == 1);
            continue;
        }
        REQUIRE(layer->external_resolution() == 3);
        const auto &subs = layer->get_region(0)->sub_slices();
        REQUIRE(subs.size() == 3);
        const double h  = layer->height;
        for (int j = 0; j < 3; ++ j) {
            const double z      = layer->slice_z - 0.5 * h + (j + 0.5) * h / 3.;
            const double r_exp  = R * (1. - z / H);
            if (r_exp < 2.)
                continue;
            REQUIRE(subs[size_t(j)].size() == 1);
            const double r_got = std::sqrt(subs[size_t(j)].front().area() * SCALING_FACTOR * SCALING_FACTOR / M_PI);
            CHECK(r_got == Approx(r_exp).margin(0.002));
            ++ checked;
        }
    }
    CHECK(checked > 100);

    // The pass externals equal the external loop of the thin print at the same plane.
    for (size_t i = 1; i < obj_thick.layer_count(); ++ i) {
        const Layer *layer = obj_thick.get_layer(int(i));
        const LayerRegion *layerm = layer->get_region(0);
        if (layerm->external_sub_passes().empty())
            continue;      // fell back (tiny top)
        REQUIRE(layerm->external_sub_passes().size() == 2);
        const double h = layer->height;
        for (int j = 0; j < 3; ++ j) {
            const double z = layer->slice_z - 0.5 * h + (j + 0.5) * h / 3.;
            const Layer *thin_layer = nullptr;
            for (const Layer *l : obj_thin.layers())
                if (std::abs(l->slice_z - z) < 1e-4)
                    thin_layer = l;
            REQUIRE(thin_layer != nullptr);
            std::vector<Points> a, b;
            if (j < 2)
                collect_loops(layerm->external_sub_passes()[size_t(j)], a);
            else
                collect_loops(layerm->perimeters(), a);
            collect_loops(thin_layer->get_region(0)->perimeters(), b);
            REQUIRE(a.size() == b.size());
            if (a.empty())
                continue;       // near the apex the external wall is an open (variable width) extrusion
            REQUIRE(a.size() == 1);
            CHECK(hausdorff(a.front(), b.front()) < 0.002);
            ++ loops_checked;
        }
    }
    CHECK(loops_checked > 100);
}

namespace {

struct GCodeBlock {
    double                z { 0. };
    double                height { 0. };
    std::set<std::string> roles;
};

// Split a G-code into the blocks started by ;LAYER_CHANGE, with the ;Z: and ;HEIGHT: of the block and the ;TYPE: roles extruded in it.
std::vector<GCodeBlock> gcode_blocks(const std::string &gcode)
{
    std::vector<GCodeBlock> blocks;
    std::istringstream      in(gcode);
    std::string             line, role;
    bool                    expect_height = false;
    while (std::getline(in, line)) {
        if (line.rfind(";LAYER_CHANGE", 0) == 0) {
            blocks.emplace_back();
            expect_height = true;
            role.clear();
        } else if (blocks.empty())
            continue;
        else if (line.rfind(";Z:", 0) == 0)
            blocks.back().z = std::stod(line.substr(3));
        else if (expect_height && line.rfind(";HEIGHT:", 0) == 0) {
            blocks.back().height = std::stod(line.substr(8));
            expect_height = false;
        } else if (line.rfind(";TYPE:", 0) == 0)
            role = line.substr(6);
        else if (line.rfind("G1 ", 0) == 0 && line.find(" E") != std::string::npos && line.find(" E-") == std::string::npos && ! role.empty())
            blocks.back().roles.insert(role);
    }
    return blocks;
}

} // namespace

TEST_CASE("External resolution emits the lower passes as real levels of the layer", "[ExternalResolution]")
{
    auto slice_cube = [](int resolution) {
        return gcode_blocks(Test::slice({ Test::TestMesh::cube_20x20x20 }, {
            { "skirts",                        0 },
            { "perimeters",                    3 },
            { "layer_height",                  0.2 },
            { "first_layer_height",            0.2 },
            { "perimeter_generator",           "arachne" },
            { "external_perimeter_resolution", resolution },
            { "cooling",                       "0" }
        }));
    };
    const std::vector<GCodeBlock> plain = slice_cube(1);
    const std::vector<GCodeBlock> hd    = slice_cube(2);
    REQUIRE(plain.size() == 100);
    // The first layer keeps k = 1, every other layer has one lower level in front of it.
    REQUIRE(hd.size() == 1 + 2 * (plain.size() - 1));
    CHECK(hd[0].z == Approx(0.2));
    for (size_t i = 1; i < plain.size(); ++ i) {
        const GCodeBlock &lower = hd[1 + 2 * (i - 1)];
        const GCodeBlock &top   = hd[2 + 2 * (i - 1)];
        // Level Z: z0 + (j + 1) h / k, the top level is at the print_z of the layer.
        CHECK(lower.z == Approx(plain[i].z - 0.1).margin(1e-4));
        CHECK(top.z   == Approx(plain[i].z).margin(1e-4));
        // The layer tag is h / k on every level.
        CHECK(lower.height == Approx(0.1).margin(1e-4));
        CHECK(top.height   == Approx(0.1).margin(1e-4));
        // The lower level carries only the external perimeter, the top level also the internals.
        CHECK(lower.roles == std::set<std::string>{ "External perimeter" });
        CHECK(top.roles.count("External perimeter") == 1);
        CHECK(top.roles.count("First internal perimeter") + top.roles.count("Second internal perimeter") + top.roles.count("Perimeter") >= 1);
    }
    // Z never decreases.
    for (size_t i = 1; i < hd.size(); ++ i)
        CHECK(hd[i].z > hd[i - 1].z);
}

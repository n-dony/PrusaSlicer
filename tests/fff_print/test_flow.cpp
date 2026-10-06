#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <numeric>
#include <cmath>
#include <vector>
#include <algorithm>
#include <sstream>

#include "test_data.hpp" // get access to init_print, etc

#include "libslic3r/Config.hpp"
#include "libslic3r/GCodeReader.hpp"
#include "libslic3r/Flow.hpp"
#include "libslic3r/libslic3r.h"

using namespace Slic3r::Test;
using namespace Slic3r;
using namespace Catch;

SCENARIO("Extrusion width specifics", "[Flow]") {

    auto test = [](const DynamicPrintConfig &config) {
        Slic3r::GCodeReader parser;
        const double        layer_height = config.opt_float("layer_height");
        std::vector<double> E_per_mm_bottom;
        parser.parse_buffer(Slic3r::Test::slice({ Slic3r::Test::TestMesh::cube_20x20x20 }, config),
            [&E_per_mm_bottom, layer_height] (Slic3r::GCodeReader& self, const Slic3r::GCodeReader::GCodeLine& line)
        { 
            if (self.z() == Approx(layer_height).margin(0.01)) { // only consider first layer
                if (line.extruding(self) && line.dist_XY(self) > 0)
                    E_per_mm_bottom.emplace_back(line.dist_E(self) / line.dist_XY(self));
            }
        });
        THEN("First layer width applies to everything on first layer.") {
            REQUIRE(E_per_mm_bottom.size() > 0);
            const double E_per_mm_avg = std::accumulate(E_per_mm_bottom.cbegin(), E_per_mm_bottom.cend(), 0.0) / static_cast<double>(E_per_mm_bottom.size());
            bool pass = (std::count_if(E_per_mm_bottom.cbegin(), E_per_mm_bottom.cend(), [E_per_mm_avg] (const double& v) { return v == Approx(E_per_mm_avg); }) == 0);
            REQUIRE(pass);
        }
        THEN("First layer width does not apply to upper layer.") {
        }
    };
    GIVEN("A config with a skirt, brim, some fill density, 3 perimeters, and 1 bottom solid layer") {
        auto config = Slic3r::DynamicPrintConfig::full_print_config_with({
            { "skirts",                         1 },
            { "brim_width",                     2 },
            { "perimeters",                     3 },
            { "fill_density",                   "40%" },
            { "first_layer_height",             0.3 },
            { "first_layer_extrusion_width",    "2" },
        });
        WHEN("Slicing a 20mm cube") {
            test(config);
        }
    }
    GIVEN("A config with more options and a 20mm cube ") {
        auto config = Slic3r::DynamicPrintConfig::full_print_config_with({
            { "skirts",                         1 },
            { "brim_width",                     2 },
            { "perimeters",                     3 },
            { "fill_density",                   "40%" },
            { "layer_height",                   "0.35" },
            { "first_layer_height",             "0.35" },
            { "bottom_solid_layers",            1 },
            { "first_layer_extrusion_width",    "2" },
            { "filament_diameter",              "3" },
            { "nozzle_diameter",                "0.5" }
        });
        WHEN("Slicing a 20mm cube") {
            test(config);            
        }
    }
}

SCENARIO(" Bridge flow specifics.", "[Flow]") {
    auto config = DynamicPrintConfig::full_print_config_with({
        { "bridge_speed",           99 },
        { "bridge_flow_ratio",      1 },
        // to prevent speeds from being altered
        { "cooling",                "0" },
        // to prevent speeds from being altered
        { "first_layer_speed",      "100%" }
    });

    auto test = [](const DynamicPrintConfig &config) {
        GCodeReader         parser;
        const double        bridge_speed = config.opt_float("bridge_speed") * 60.;
        std::vector<double> E_per_mm;
        parser.parse_buffer(Slic3r::Test::slice({ Slic3r::Test::TestMesh::overhang }, config), 
            [&E_per_mm, bridge_speed](Slic3r::GCodeReader &self, const Slic3r::GCodeReader::GCodeLine &line) {
            if (line.extruding(self) && line.dist_XY(self) > 0) {
                if (is_approx<double>(line.new_F(self), bridge_speed))
                    E_per_mm.emplace_back(line.dist_E(self) / line.dist_XY(self));
            }
        });
        const double nozzle_dmr                 = config.opt<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
        const double filament_dmr               = config.opt<ConfigOptionFloats>("filament_diameter")->get_at(0);
        const double bridge_mm_per_mm           = sqr(nozzle_dmr / filament_dmr) * config.opt_float("bridge_flow_ratio");
        size_t num_errors = std::count_if(E_per_mm.begin(), E_per_mm.end(), 
            [bridge_mm_per_mm](double v){ return std::abs(v - bridge_mm_per_mm) > 0.01; });
        return num_errors == 0;
    };

    GIVEN("A default config with no cooling and a fixed bridge speed, flow ratio and an overhang mesh.") {
        WHEN("bridge_flow_ratio is set to 0.5 and extrusion width to default") {
            config.set_deserialize_strict({ { "bridge_flow_ratio", 0.5}, { "extrusion_width", "0" } });
            THEN("Output flow is as expected.") {
                REQUIRE(test(config));
            }
        }
        WHEN("bridge_flow_ratio is set to 2.0 and extrusion width to default") {
            config.set_deserialize_strict({ { "bridge_flow_ratio", 2.0}, { "extrusion_width", "0" } });
            THEN("Output flow is as expected.") {
                REQUIRE(test(config));
            }
        }
        WHEN("bridge_flow_ratio is set to 0.5 and extrusion_width to 0.4") {
            config.set_deserialize_strict({ { "bridge_flow_ratio", 0.5}, { "extrusion_width", 0.4 } });
            THEN("Output flow is as expected.") {
                REQUIRE(test(config));
            }
        }
        WHEN("bridge_flow_ratio is set to 1.0 and extrusion_width to 0.4") {
            config.set_deserialize_strict({ { "bridge_flow_ratio", 1.0}, { "extrusion_width", 0.4 } });
            THEN("Output flow is as expected.") {
                REQUIRE(test(config));
            }
        }
        WHEN("bridge_flow_ratio is set to 2 and extrusion_width to 0.4") {
            config.set_deserialize_strict({ { "bridge_flow_ratio", 2.}, { "extrusion_width", 0.4 } });
            THEN("Output flow is as expected.") {
                REQUIRE(test(config));
            }
        }
    }
    GIVEN("A default config with no cooling and a fixed bridge speed, flow ratio, fixed extrusion width of 0.4mm and an overhang mesh.") {
        WHEN("bridge_flow_ratio is set to 1.0") {
            THEN("Output flow is as expected.") {
            }
        }
        WHEN("bridge_flow_ratio is set to 0.5") {
            THEN("Output flow is as expected.") {
            }
        }
        WHEN("bridge_flow_ratio is set to 2.0") {
            THEN("Output flow is as expected.") {
            }
        }
    }
}

/// Test the expected behavior for auto-width, 
/// spacing, etc
SCENARIO("Flow: Flow math for non-bridges", "[Flow]") {
    GIVEN("Nozzle Diameter of 0.4, a desired width of 1mm and layer height of 0.5") {
        ConfigOptionFloatOrPercent	width(1.0, false);
        float nozzle_diameter	= 0.4f;
        float layer_height		= 0.4f;

        // Spacing for non-bridges is has some overlap
        THEN("External perimeter flow has spacing fixed to 1.125 * nozzle_diameter") {
            auto flow = Flow::new_from_config_width(frExternalPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Approx(1.125 * nozzle_diameter - layer_height * (1.0 - PI / 4.0)));
        }

        THEN("Internal perimeter flow has spacing fixed to 1.125 * nozzle_diameter") {
            auto flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Approx(1.125 *nozzle_diameter - layer_height * (1.0 - PI / 4.0)));
        }
        THEN("Spacing for supplied width is 0.8927f") {
            auto flow = Flow::new_from_config_width(frExternalPerimeter, width, nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Approx(width.value - layer_height * (1.0 - PI / 4.0)));
            flow = Flow::new_from_config_width(frPerimeter, width, nozzle_diameter, layer_height);
            REQUIRE(flow.spacing() == Approx(width.value - layer_height * (1.0 - PI / 4.0)));
        }
    }
    /// Check the min/max
    GIVEN("Nozzle Diameter of 0.25") {
        float nozzle_diameter	= 0.25f;
        float layer_height		= 0.5f;
        WHEN("layer height is set to 0.2") {
            layer_height = 0.15f;
            THEN("Max width is set.") {
                auto flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
                REQUIRE(flow.width() == Approx(1.125 * nozzle_diameter));
            }
        }
        WHEN("Layer height is set to 0.25") {
            layer_height = 0.25f;
            THEN("Min width is set.") {
                auto flow = Flow::new_from_config_width(frPerimeter, ConfigOptionFloatOrPercent(0, false), nozzle_diameter, layer_height);
                REQUIRE(flow.width() == Approx(1.125 * nozzle_diameter));
            }
        }
    }

#if 0
    /// Check for an edge case in the maths where the spacing could be 0; original
    /// math is 0.99. Slic3r issue #4654
    GIVEN("Input spacing of 0.414159 and a total width of 2") {
        double in_spacing = 0.414159;
        double total_width = 2.0;
        auto flow = Flow::new_from_spacing(1.0, 0.4, 0.3);
        WHEN("solid_spacing() is called") {
            double result = flow.solid_spacing(total_width, in_spacing);
            THEN("Yielded spacing is greater than 0") {
                REQUIRE(result > 0);
            }
        }
    }
#endif    

}

/// Spacing, width calculation for bridge extrusions
SCENARIO("Flow: Flow math for bridges", "[Flow]") {
    GIVEN("Nozzle Diameter of 0.4, a desired width of 1mm and layer height of 0.5") {
		float nozzle_diameter	= 0.4f;
		float bridge_flow		= 1.0f;
        WHEN("Flow role is frExternalPerimeter") {
            auto flow = Flow::bridging_flow(nozzle_diameter * sqrt(bridge_flow), nozzle_diameter);
            THEN("Bridge width is same as nozzle diameter") {
                REQUIRE(flow.width() == Approx(nozzle_diameter));
            }
            THEN("Bridge spacing is same as nozzle diameter + BRIDGE_EXTRA_SPACING") {
                REQUIRE(flow.spacing() == Approx(nozzle_diameter + BRIDGE_EXTRA_SPACING));
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Fork: combine_perimeters flow model. Pure arithmetic and cross-section raster
// tests, no slicing.
//
// A bead of width w and height h has the stadium cross-section: a rectangle
// (w - h) x h with semicircular ends of radius h/2. Its area is
// A = h * (w - h * (1 - pi/4)) and its extrusion spacing is s = w - h * (1 - pi/4).
// A combined bead of height H = N * h keeps the spacing and gets the width
// w_H = w + (H - h) * (1 - pi/4), so that its area is H * s = N * A.
// ---------------------------------------------------------------------------

namespace {

constexpr double k_stadium = 1. - M_PI / 4.;

double stadium_spacing(double w, double h) { return w - h * k_stadium; }
double stadium_area(double w, double h)    { return h * stadium_spacing(w, h); }
double merged_width(double w, double h, double H) { return w + (H - h) * k_stadium; }

// Half width of a stadium of width w, height h at local height z (negative if outside).
double stadium_halfwidth(double z, double w, double h)
{
    if (z < 0. || z > h)
        return -1.;
    const double r = h / 2.;
    return (w - h) / 2. + std::sqrt(std::max(0., r * r - (z - r) * (z - r)));
}

struct Bead { double xc, z0, w, h; };

// Boolean raster of a set of stadium beads, rows = z samples, columns = x samples.
struct Raster {
    double              res;
    double              x0;
    std::vector<double> xs, zs;
    std::vector<char>   m;
    char&       at(size_t iz, size_t ix)       { return m[iz * xs.size() + ix]; }
    char        at(size_t iz, size_t ix) const { return m[iz * xs.size() + ix]; }
};

Raster rasterise(const std::vector<Bead> &beads, double res, double H)
{
    Raster r;
    r.res = res;
    r.x0  = -1.2;
    for (double x = -1.2; x < 1.2; x += res)
        r.xs.push_back(x);
    for (double z = res / 2.; z < H; z += res)
        r.zs.push_back(z);
    r.m.assign(r.xs.size() * r.zs.size(), 0);
    for (const Bead &b : beads)
        for (size_t iz = 0; iz < r.zs.size(); ++ iz) {
            const double hw = stadium_halfwidth(r.zs[iz] - b.z0, b.w, b.h);
            if (hw < 0.)
                continue;
            for (size_t ix = 0; ix < r.xs.size(); ++ ix)
                if (std::abs(r.xs[ix] - b.xc) <= hw)
                    r.at(iz, ix) = 1;
        }
    return r;
}

double raster_area(const Raster &r)
{
    size_t n = 0;
    for (char c : r.m)
        n += c;
    return double(n) * r.res * r.res;
}

double raster_symdiff(const Raster &a, const Raster &b)
{
    size_t n = 0;
    for (size_t i = 0; i < a.m.size(); ++ i)
        n += (a.m[i] != b.m[i]);
    return double(n) * a.res * a.res;
}

struct CrossSection {
    double area_thin, area_merged, eps;
    // Worst extra void width (mm) over the layers, merged bead anchored at the top layer position.
    double worst_extra_void;
};

// N thin beads (w, h) staggered laterally by delta per layer (top layer at x = 0) versus one merged
// bead of height N*h at the top layer position. External and second-internal neighbours stay thin;
// the "extra void" is the void area the merged bead leaves in the band of each layer compared to the
// thin stack, divided by the layer height.
CrossSection cross_section(double w, double h, int N, double delta, double res = 0.002)
{
    const double H  = N * h;
    const double s  = stadium_spacing(w, h);
    const double wH = merged_width(w, h, H);
    std::vector<double> d(N);
    for (int k = 0; k < N; ++ k)
        d[k] = - double(N - 1 - k) * delta;

    std::vector<Bead> thin, neighbours;
    for (int k = 0; k < N; ++ k) {
        thin.push_back({ d[k], k * h, w, h });
        neighbours.push_back({ d[k] - s, k * h, w, h });
        neighbours.push_back({ d[k] + s, k * h, w, h });
    }
    const Raster mt = rasterise(thin, res, H);
    const Raster mm = rasterise({ { 0., 0., wH, H } }, res, H);
    const Raster mn = rasterise(neighbours, res, H);

    CrossSection out;
    out.area_thin   = raster_area(mt);
    out.area_merged = raster_area(mm);
    out.eps         = raster_symdiff(mt, mm) / (H * s);
    out.worst_extra_void = 0.;
    for (int k = 0; k < N; ++ k) {
        const double lo = d[k] - s - w / 2., hi = d[k] + s + w / 2.;
        double void_merged = 0., void_thin = 0.;
        for (size_t iz = 0; iz < mm.zs.size(); ++ iz) {
            if (mm.zs[iz] < k * h || mm.zs[iz] >= (k + 1) * h)
                continue;
            for (size_t ix = 0; ix < mm.xs.size(); ++ ix) {
                if (mm.xs[ix] < lo || mm.xs[ix] > hi)
                    continue;
                if (! (mn.at(iz, ix) || mm.at(iz, ix)))
                    void_merged += res * res;
                if (! (mn.at(iz, ix) || mt.at(iz, ix)))
                    void_thin += res * res;
            }
        }
        out.worst_extra_void = std::max(out.worst_extra_void, (void_merged - void_thin) / h);
    }
    return out;
}

} // namespace

SCENARIO("Fork: combine_perimeters stadium flow identities", "[Flow][Perimeters]")
{
    // Pure arithmetic of the per-path flow model (no slicing).
    for (double w : { 0.35, 0.45, 0.5, 0.6 })
        for (double h : { 0.05, 0.1, 0.15, 0.2 })
            for (int N : { 2, 3, 4 }) {
                const double H  = N * h;
                const double wH = merged_width(w, h, H);
                // The library spacing function matches the stadium model.
                REQUIRE(Flow::rounded_rectangle_extrusion_spacing(float(w), float(h)) == Approx(stadium_spacing(w, h)).margin(1e-6));
                // Spacing is preserved by the merge.
                REQUIRE(stadium_spacing(wH, H) == Approx(stadium_spacing(w, h)).margin(1e-12));
                // The merged bead carries exactly N times the thin area (volume conservation).
                REQUIRE(stadium_area(wH, H) == Approx(N * stadium_area(w, h)).margin(1e-12));
                // The library flow agrees with the model for mm3_per_mm.
                const Flow thin(float(w), float(h), 0.4f);
                REQUIRE(thin.mm3_per_mm() == Approx(stadium_area(w, h)).margin(1e-6));
                const Flow merged(float(wH), float(H), 0.4f);
                REQUIRE(merged.mm3_per_mm() == Approx(N * thin.mm3_per_mm()).margin(1e-5));
            }
}

SCENARIO("Fork: combine_perimeters per-path width cap", "[Flow][Perimeters]")
{
    // A merged bead must be at least as wide as it is tall (width >= height). For a thin Arachne
    // path (w = 0.30, h = 0.1) N = 4 (H = 0.4) violates that, N = 2 does not; the default 0.45
    // allows N = 4.
    const double h = 0.1;
    GIVEN("thin path w=0.30") {
        REQUIRE(merged_width(0.30, h, 4 * h) < 4 * h);
        REQUIRE(merged_width(0.30, h, 2 * h) >= 2 * h);
        REQUIRE(merged_width(0.30, h, 3 * h) >= 3 * h);
    }
    GIVEN("default path w=0.45") {
        REQUIRE(merged_width(0.45, h, 4 * h) >= 4 * h);
    }
}

SCENARIO("Fork: combine_perimeters cross-section raster", "[Flow][Perimeters]")
{
    const double w = 0.45, h = 0.1;
    struct Floor { int N; double eps0_percent; };

    GIVEN("coincident beads (delta = 0)") {
        for (Floor f : { Floor{ 2, 8.0 }, Floor{ 3, 13.5 }, Floor{ 4, 18.0 } }) {
            const CrossSection cs = cross_section(w, h, f.N, 0.);
            // (a) merged area equals N * thin area within 0.5%.
            CHECK(std::abs(cs.area_merged / cs.area_thin - 1.) < 0.005);
            // The model area of the thin stack matches the raster.
            CHECK(std::abs(cs.area_thin / (f.N * stadium_area(w, h)) - 1.) < 0.005);
            // Shape floor: symmetric difference / merged area (percent points).
            CHECK(std::abs(cs.eps * 100. - f.eps0_percent) < 0.5);
            // Shape residual without lateral shift (measured 0.02 mm for N=3, 0.05 mm for N=4, 0 for N=2).
            CHECK(cs.worst_extra_void < 0.06);
        }
    }
    GIVEN("laterally shifted beads") {
        for (int N : { 2, 4 }) {
            const double eps0 = cross_section(w, h, N, 0.).eps;
            double prev = -1.;
            for (double delta : { 0.03, 0.05, 0.075, 0.1 }) {
                const CrossSection cs = cross_section(w, h, N, delta);
                const double D          = (N - 1) * delta;
                const double closed     = std::max(0., D - h * k_stadium);
                // Weaker than the +-20% closed-form claim, which only holds for N=2 and D >= 0.05
                // (measured: raster 16.4/34.2/57.2/82.4 um versus closed form 8.5/28.5/53.5/78.5 um for
                // N=2 and delta = .03/.05/.075/.1; for N=4 the raster exceeds the closed form by up to
                // 43 um because the stack of beads also drifts laterally between layers). The
                // closed form is a LOWER bound of the raster and the excess stays small.
                CHECK(cs.worst_extra_void >= closed * 0.99);
                CHECK(cs.worst_extra_void - closed < 0.05);
                // Monotone increase with the shift.
                CHECK(cs.worst_extra_void > prev);
                prev = cs.worst_extra_void;
                // Gap is positive above h*(1-pi/4) of drift.
                if (D > h * k_stadium)
                    CHECK(cs.worst_extra_void > 0.);
                // raster eps - eps0 <= rectangle bound (N-1)*delta/w.
                CHECK(cs.eps - eps0 <= D / w + 1e-3);
            }
        }
        // N=2, D >= 0.05: the closed form is within +-20% of the raster.
        for (double delta : { 0.075, 0.1 }) {
            const CrossSection cs = cross_section(w, h, 2, delta);
            const double closed = delta - h * k_stadium;
            CHECK(std::abs(cs.worst_extra_void / closed - 1.) < 0.2);
        }
    }
}

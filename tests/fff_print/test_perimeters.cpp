#include <catch2/catch_test_macros.hpp>

#include <numeric>
#include <cmath>
#include <sstream>
#include <set>
#include <tuple>
#include <chrono>
#include <limits>
#include <algorithm>
#include <array>
#include <catch2/catch_approx.hpp>

#include "libslic3r/Config.hpp"
#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/PerimeterGenerator.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/Preset.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/SurfaceCollection.hpp"
#include "libslic3r/libslic3r.h"

#include "test_data.hpp"

using namespace Slic3r;
using Catch::Approx;

SCENARIO("Perimeter nesting", "[Perimeters]")
{
    struct TestData {
        ExPolygons          expolygons;
        // expected number of loops
        int                 total;
        // expected number of external loops
        int                 external;
        // expected external perimeter
        std::vector<bool>   ext_order;
        // expected number of internal contour loops
        int                 cinternal;
        // expected number of ccw loops
        int                 ccw;
        // expected ccw/cw order
        std::vector<bool>   ccw_order;
        // expected nesting order
        std::vector<std::vector<int>> nesting;
    };

    FullPrintConfig config;

    auto test = [&config](const TestData &data) {
        SurfaceCollection slices;
        slices.append(data.expolygons, stInternal);
        
        ExtrusionEntityCollection loops;
        ExtrusionEntityCollection gap_fill;
        ExPolygons                fill_expolygons;
        Flow                      flow(1., 1., 1.);
        PerimeterRegions          perimeter_regions;
        PerimeterGenerator::Parameters perimeter_generator_params(
            1., // layer height
            -1, // layer ID
            flow, flow, flow, flow, flow, flow,
            static_cast<const PrintRegionConfig&>(config),
            static_cast<const PrintObjectConfig&>(config),
            static_cast<const PrintConfig&>(config),
            perimeter_regions,
            false); // spiral_vase
        Polygons lower_layer_polygons_cache;
        for (const Surface &surface : slices)
        // FIXME Lukas H.: Disable this test for Arachne because it is failing and needs more investigation.
//        if (config.perimeter_generator == PerimeterGeneratorType::Arachne)
//            PerimeterGenerator::process_arachne();
//        else
            PerimeterGenerator::process_classic(
                // input:
                perimeter_generator_params,
                surface,
                nullptr,
                nullptr,
                // cache:
                lower_layer_polygons_cache,
                // output:
                loops, gap_fill, fill_expolygons);

        THEN("expected number of collections") {
            REQUIRE(loops.entities.size() == data.expolygons.size());
        }        
        
        loops = loops.flatten();
        THEN("expected number of loops") {
            REQUIRE(loops.entities.size() == data.total);
        }
        THEN("expected number of external loops") {
            size_t num_external = std::count_if(loops.entities.begin(), loops.entities.end(), 
                [](const ExtrusionEntity *ee){ return ee->role() == ExtrusionRole::ExternalPerimeter; });
            REQUIRE(num_external == data.external);
        }
        THEN("expected external order") {
            std::vector<bool> ext_order;
            for (auto *ee : loops.entities)
                ext_order.emplace_back(ee->role() == ExtrusionRole::ExternalPerimeter);
            REQUIRE(ext_order == data.ext_order);
        }
        THEN("expected number of internal contour loops") {
            size_t cinternal = std::count_if(loops.entities.begin(), loops.entities.end(), 
                [](const ExtrusionEntity *ee){ return dynamic_cast<const ExtrusionLoop*>(ee)->loop_role() == elrContourInternalPerimeter; });
            REQUIRE(cinternal == data.cinternal);
        }
        THEN("expected number of ccw loops") {
            size_t ccw = std::count_if(loops.entities.begin(), loops.entities.end(), 
                [](const ExtrusionEntity *ee){ return dynamic_cast<const ExtrusionLoop*>(ee)->polygon().is_counter_clockwise(); });
            REQUIRE(ccw == data.ccw);
        }
        THEN("expected ccw/cw order") {
            std::vector<bool> ccw_order;
            for (auto *ee : loops.entities)
                ccw_order.emplace_back(dynamic_cast<const ExtrusionLoop*>(ee)->polygon().is_counter_clockwise());
            REQUIRE(ccw_order == data.ccw_order);
        }
        THEN("expected nesting order") {
            for (const std::vector<int> &nesting : data.nesting) {
                for (size_t i = 1; i < nesting.size(); ++ i)
                    REQUIRE(dynamic_cast<const ExtrusionLoop*>(loops.entities[nesting[i - 1]])->polygon().contains(loops.entities[nesting[i]]->first_point()));
            }
        }
    };

    WHEN("Rectangle") {
        config.perimeters.value = 3;
        TestData data;
        data.expolygons  = { 
            ExPolygon{ Polygon::new_scale({ {0,0}, {100,0}, {100,100}, {0,100} }) }
        };
        data.total       = 3;
        data.external    = 1;
        data.ext_order   = { false, false, true };
        data.cinternal   = 1;
        data.ccw         = 3;
        data.ccw_order   = { true, true, true };
        data.nesting     = { { 2, 1, 0 } };
        test(data);
    }
    WHEN("Rectangle with hole") {
        config.perimeters.value = 3;
        TestData data;
        data.expolygons  = { 
            ExPolygon{ Polygon::new_scale({ {0,0}, {100,0}, {100,100}, {0,100} }), 
                       Polygon::new_scale({ {40,40}, {40,60}, {60,60}, {60,40} }) } 
        };
        data.total       = 6;
        data.external    = 2;
        data.ext_order   = { false, false, true, false, false, true };
        data.cinternal   = 1;
        data.ccw         = 3;
        data.ccw_order   = { false, false, false, true, true, true };
        data.nesting     = { { 5, 4, 3, 0, 1, 2 } };
        test(data);
    }
    WHEN("Nested rectangles with holes") {
        config.perimeters.value = 3;
        TestData data;
        data.expolygons  = {
            ExPolygon{ Polygon::new_scale({ {0,0}, {200,0}, {200,200}, {0,200} }), 
                       Polygon::new_scale({ {20,20}, {20,180}, {180,180}, {180,20} }) },
            ExPolygon{ Polygon::new_scale({ {50,50}, {150,50}, {150,150}, {50,150} }), 
                       Polygon::new_scale({ {80,80}, {80,120}, {120,120}, {120,80} }) }
        };
        data.total       = 4*3;
        data.external    = 4;
        data.ext_order   = { false, false, true, false, false, true, false, false, true, false, false, true };
        data.cinternal   = 2;
        data.ccw         = 2*3;
        data.ccw_order   = { false, false, false, true, true, true, false, false, false, true, true, true };
        test(data);
    }
    WHEN("Rectangle with multiple holes") {
        config.perimeters.value = 2;
        TestData data;
        ExPolygon expoly{ Polygon::new_scale({ {0,0}, {50,0}, {50,50}, {0,50} }) };
        expoly.holes.emplace_back(Polygon::new_scale({ {7.5,7.5},  {7.5,12.5},  {12.5,12.5}, {12.5,7.5}  }));
        expoly.holes.emplace_back(Polygon::new_scale({ {7.5,17.5}, {7.5,22.5},  {12.5,22.5}, {12.5,17.5} }));
        expoly.holes.emplace_back(Polygon::new_scale({ {7.5,27.5}, {7.5,32.5},  {12.5,32.5}, {12.5,27.5} }));
        expoly.holes.emplace_back(Polygon::new_scale({ {7.5,37.5}, {7.5,42.5},  {12.5,42.5}, {12.5,37.5} }));
        expoly.holes.emplace_back(Polygon::new_scale({ {17.5,7.5}, {17.5,12.5}, {22.5,12.5}, {22.5,7.5}  }));
        data.expolygons  = { expoly };
        data.total       = 12;
        data.external    = 6;
        data.ext_order   = { false, true, false, true, false, true, false, true, false, true, false, true };
        data.cinternal   = 1;
        data.ccw         = 2;
        data.ccw_order   = { false, false, false, false, false, false, false, false, false, false, true, true };
        data.nesting     = { {0,1},{2,3},{4,5},{6,7},{8,9} };
        test(data);
    };
}

SCENARIO("Perimeters", "[Perimeters]")
{
    auto config = Slic3r::DynamicPrintConfig::full_print_config_with({
        { "skirts",                 0 },
        { "fill_density",           0 },
        { "perimeters",             3 },
        { "top_solid_layers",       0 },
        { "bottom_solid_layers",    0 },
        // to prevent speeds from being altered
        { "cooling",                "0" },
        // to prevent speeds from being altered
        { "first_layer_speed",      "100%" }
    });

    WHEN("Bridging perimeters disabled") {
        std::string gcode = Slic3r::Test::slice({ Slic3r::Test::TestMesh::overhang }, config);

        THEN("all perimeters extruded ccw") {
            GCodeReader parser;
            bool        has_cw_loops = false;
            Polygon     current_loop;
            parser.parse_buffer(gcode, [&has_cw_loops, &current_loop](Slic3r::GCodeReader &self, const Slic3r::GCodeReader::GCodeLine &line)
            {
                if (line.extruding(self) && line.dist_XY(self) > 0) {
                    if (current_loop.empty())
                        current_loop.points.emplace_back(self.xy_scaled());
                    current_loop.points.emplace_back(line.new_XY_scaled(self));
                } else if (! line.cmd_is("M73")) {
                    // skips remaining time lines (M73)
                    if (! current_loop.empty() && current_loop.is_clockwise())
                        has_cw_loops = true;
                    current_loop.clear();
                }
            });
            REQUIRE(! has_cw_loops);
        }
    }
    
    auto test = [&config](Test::TestMesh model) {    
        // we test two copies to make sure ExtrusionLoop objects are not modified in-place (the second object would not detect cw loops and thus would calculate wrong)
        std::string gcode = Slic3r::Test::slice({ model, model }, config);
        GCodeReader parser;
        bool        has_cw_loops = false;
        bool        has_outwards_move = false;
        bool        starts_on_convex_point = false;
        // print_z => count of external loops
        std::map<coord_t, int> external_loops;
        Polygon     current_loop;
        const double external_perimeter_speed = config.get_abs_value("external_perimeter_speed") * 60.;
        parser.parse_buffer(gcode, [&has_cw_loops, &has_outwards_move, &starts_on_convex_point, &external_loops, &current_loop, external_perimeter_speed, model]
            (Slic3r::GCodeReader &self, const Slic3r::GCodeReader::GCodeLine &line)
        {
            if (line.extruding(self) && line.dist_XY(self) > 0) {
                if (current_loop.empty())
                    current_loop.points.emplace_back(self.xy_scaled());
                current_loop.points.emplace_back(line.new_XY_scaled(self));
            } else if (! line.cmd_is("M73")) {
                // skips remaining time lines (M73)
                if (! current_loop.empty()) {
                    if (current_loop.is_clockwise())
                        has_cw_loops = true;
                    if (is_approx<double>(self.f(), external_perimeter_speed)) {
                        // reset counter for second object
                        coord_t z = scaled<coord_t>(self.z());
                        auto it = external_loops.find(z);
                        if (it == external_loops.end())
                            it = external_loops.insert(std::make_pair(z, 0)).first;
                        else if (it->second == 2)
                            it->second = 0;
                        ++ it->second;
                        bool is_contour          = it->second == 2;
                        bool is_hole             = it->second == 1;
                        // Testing whether the move point after loop ends up inside the extruded loop.
                        bool loop_contains_point = current_loop.contains(line.new_XY_scaled(self));
                        if (// contour should include destination
                            (! loop_contains_point && is_contour) ||
                            // hole should not
                            (loop_contains_point && is_hole))
                            has_outwards_move = true;
                        if (model == Test::TestMesh::cube_with_concave_hole) {
                            // check that loop starts at a concave vertex
                            double cross = cross2((current_loop.points.front() - current_loop.points[current_loop.points.size() - 2]).cast<double>(), (current_loop.points[1] - current_loop.points.front()).cast<double>());
                            bool   convex = cross > 0.;
                            if ((convex && is_contour) || (! convex && is_hole))
                                starts_on_convex_point = true;
                        }
                    }
                    current_loop.clear();
                }
            }
        });
        THEN("all perimeters extruded ccw") {
            REQUIRE(! has_cw_loops);
        }

        // FIXME Lukas H.: Arachne is printing external loops before hole loops in this test case.
        if (config.opt_enum<PerimeterGeneratorType>("perimeter_generator") == Slic3r::PerimeterGeneratorType::Arachne) {
            THEN("move outwards after completing external loop") {
//                REQUIRE(! has_outwards_move);
            }
            // FIXME Lukas H.: Disable this test for Arachne because it is failing and needs more investigation.
            THEN("loops start on concave point if any") {
//                REQUIRE(! starts_on_convex_point);
            }
        } else {
            THEN("move inwards after completing external loop") {
                REQUIRE(! has_outwards_move);
            }
            THEN("loops start on concave point if any") {
                REQUIRE(! starts_on_convex_point);
            }
        }

    };
    // Reusing the config above.
    config.set_deserialize_strict({
        { "external_perimeter_speed", 68 }
    });
    GIVEN("Cube with hole") { test(Test::TestMesh::cube_with_hole); }
    GIVEN("Cube with concave hole") { test(Test::TestMesh::cube_with_concave_hole); }
    
    WHEN("Bridging perimeters enabled") {
        // Reusing the config above.
        config.set_deserialize_strict({
            { "perimeters",                 1 },
            { "perimeter_speed",            77 },
            { "external_perimeter_speed",   66 },
            { "first_internal_perimeter_speed",   66 },
            { "second_internal_perimeter_speed",   66 },
            { "enable_dynamic_overhang_speeds", false },
            { "bridge_speed",               99 },
            { "cooling",                    "1" },
            { "fan_below_layer_time",       "0" },
            { "slowdown_below_layer_time",  "0" },
            { "bridge_fan_speed",           "100" },
            // arbitrary value
            { "bridge_flow_ratio",          33 },
            { "overhangs",                  true }
        });
    
        std::string gcode = Slic3r::Test::slice({ mesh(Slic3r::Test::TestMesh::overhang) }, config);

        THEN("Bridging is applied to bridging perimeters") {
            GCodeReader  parser;
            // print Z => speeds
            std::map<coord_t, std::set<double>> layer_speeds;
            int          fan_speed = 0;
            const double perimeter_speed            = config.opt_float("perimeter_speed") * 60.;
            const double external_perimeter_speed   = config.get_abs_value("external_perimeter_speed") * 60.;
            const double bridge_speed               = config.opt_float("bridge_speed") * 60.;
            const double nozzle_dmr                 = config.opt<ConfigOptionFloats>("nozzle_diameter")->get_at(0);
            const double filament_dmr               = config.opt<ConfigOptionFloats>("filament_diameter")->get_at(0);
            const double bridge_mm_per_mm           = sqr(nozzle_dmr / filament_dmr) * config.opt_float("bridge_flow_ratio");
            parser.parse_buffer(gcode, [&layer_speeds, &fan_speed, perimeter_speed, external_perimeter_speed, bridge_speed, nozzle_dmr, filament_dmr, bridge_mm_per_mm]
                (Slic3r::GCodeReader &self, const Slic3r::GCodeReader::GCodeLine &line)
            {
                if (line.cmd_is("M107"))
                    fan_speed = 0;
                else if (line.cmd_is("M106"))
                    line.has_value('S', fan_speed);
                else if (line.extruding(self) && line.dist_XY(self) > 0) {
                    double feedrate = line.new_F(self);
                    REQUIRE((is_approx(feedrate, perimeter_speed) || is_approx(feedrate, external_perimeter_speed) || is_approx(feedrate, bridge_speed)));
                    layer_speeds[self.z()].insert(feedrate);
                    bool   bridging  = is_approx(feedrate, bridge_speed);
                    double mm_per_mm = line.dist_E(self) / line.dist_XY(self);
                    // Fan enabled at full speed when bridging, disabled when not bridging.
                    REQUIRE((! bridging || fan_speed == 255));
                    REQUIRE((bridging || fan_speed == 0));
                    // When bridging, bridge flow is applied.
                    REQUIRE((! bridging || std::abs(mm_per_mm - bridge_mm_per_mm) <= 0.01));
                }
            });
            // only overhang layer has more than one speed
            size_t num_overhangs = std::count_if(layer_speeds.begin(), layer_speeds.end(), [](const std::pair<double, std::set<double>> &v){ return v.second.size() > 1; });
            REQUIRE(num_overhangs == 1);
        }
    }

    GIVEN("iPad stand") {
        WHEN("Extra perimeters enabled") {
            auto config = Slic3r::DynamicPrintConfig::full_print_config_with({
                { "skirts",                     0 },
                { "perimeters",                 3 },
                { "layer_height",               0.4 },
                { "first_layer_height",         0.35 },
                { "extra_perimeters",           1 },
                // to prevent speeds from being altered
                { "cooling",                    "0" },
                // to prevent speeds from being altered
                { "first_layer_speed",          "100%" },
                { "perimeter_speed",            99 },
                { "external_perimeter_speed",   99 },
                { "small_perimeter_speed",      99 },
                { "thin_walls",                 0 },
            });
        
            std::string gcode = Slic3r::Test::slice({ Slic3r::Test::TestMesh::ipadstand }, config);
            // z => number of loops
            std::map<coord_t, int> perimeters;
            bool                   in_loop         = false;
            const double           perimeter_speed = config.opt_float("perimeter_speed") * 60.;
            GCodeReader            parser;
            parser.parse_buffer(gcode, [&perimeters, &in_loop, perimeter_speed](Slic3r::GCodeReader &self, const Slic3r::GCodeReader::GCodeLine &line)
            {
                if (line.extruding(self) && line.dist_XY(self) > 0 && is_approx<double>(line.new_F(self), perimeter_speed)) {
                    if (! in_loop) {
                        coord_t z = scaled<coord_t>(self.z());
                        auto it = perimeters.find(z);
                        if (it == perimeters.end())
                            it = perimeters.insert(std::make_pair(z, 0)).first;
                        ++ it->second;
                    }
                    in_loop = true;
                } else if (! line.cmd_is("M73")) {
                    // skips remaining time lines (M73)
                    in_loop = false;
                }
            });
            THEN("no superfluous extra perimeters") {
                const int num_perimeters = config.opt_int("perimeters");
                size_t extra_perimeters = std::count_if(perimeters.begin(), perimeters.end(), [num_perimeters](const std::pair<const coord_t, int> &v){ return (v.second % num_perimeters) > 0; });
                REQUIRE(extra_perimeters == 0);
            }
        }
    }
}

SCENARIO("Some weird coverage test", "[Perimeters]")
{
    auto config = Slic3r::DynamicPrintConfig::full_print_config_with({
        { "nozzle_diameter",                    "0.4" },
        { "perimeters",                         2 },
        { "perimeter_extrusion_width",          0.4 },
        { "external_perimeter_extrusion_width", 0.4 },
        { "infill_extrusion_width",             0.53 },
        { "solid_infill_extrusion_width",       0.53 }
    });

    // we just need a pre-filled Print object
    Print print;
    Model model;
    Slic3r::Test::init_print({ Test::TestMesh::cube_20x20x20 }, print, model, config);
    
    // override a layer's slices
    ExPolygon expolygon;
    expolygon.contour = {
        {-71974463,-139999376},{-71731792,-139987456},{-71706544,-139985616},{-71682119,-139982639},{-71441248,-139946912},{-71417487,-139942895},{-71379384,-139933984},{-71141800,-139874480},
        {-71105247,-139862895},{-70873544,-139779984},{-70838592,-139765856},{-70614943,-139660064},{-70581783,-139643567},{-70368368,-139515680},{-70323751,-139487872},{-70122160,-139338352},
        {-70082399,-139306639},{-69894800,-139136624},{-69878679,-139121327},{-69707992,-138933008},{-69668575,-138887343},{-69518775,-138685359},{-69484336,-138631632},{-69356423,-138418207},
        {-69250040,-138193296},{-69220920,-138128976},{-69137992,-137897168},{-69126095,-137860255},{-69066568,-137622608},{-69057104,-137582511},{-69053079,-137558751},{-69017352,-137317872},
        {-69014392,-137293456},{-69012543,-137268207},{-68999369,-137000000},{-63999999,-137000000},{-63705947,-136985551},{-63654984,-136977984},{-63414731,-136942351},{-63364756,-136929840},
        {-63129151,-136870815},{-62851950,-136771631},{-62585807,-136645743},{-62377483,-136520895},{-62333291,-136494415},{-62291908,-136463728},{-62096819,-136319023},{-62058644,-136284432},
        {-61878676,-136121328},{-61680968,-135903184},{-61650275,-135861807},{-61505591,-135666719},{-61354239,-135414191},{-61332211,-135367615},{-61228359,-135148063},{-61129179,-134870847},
        {-61057639,-134585262},{-61014451,-134294047},{-61000000,-134000000},{-61000000,-107999999},{-61014451,-107705944},{-61057639,-107414736},{-61129179,-107129152},{-61228359,-106851953},
        {-61354239,-106585808},{-61505591,-106333288},{-61680967,-106096816},{-61878675,-105878680},{-62096820,-105680967},{-62138204,-105650279},{-62333292,-105505591},{-62585808,-105354239},
        {-62632384,-105332207},{-62851951,-105228360},{-62900463,-105211008},{-63129152,-105129183},{-63414731,-105057640},{-63705947,-105014448},{-63999999,-105000000},{-68999369,-105000000},
        {-69012543,-104731792},{-69014392,-104706544},{-69017352,-104682119},{-69053079,-104441248},{-69057104,-104417487},{-69066008,-104379383},{-69125528,-104141799},{-69137111,-104105248},
        {-69220007,-103873544},{-69234136,-103838591},{-69339920,-103614943},{-69356415,-103581784},{-69484328,-103368367},{-69512143,-103323752},{-69661647,-103122160},{-69693352,-103082399},
        {-69863383,-102894800},{-69878680,-102878679},{-70066999,-102707992},{-70112656,-102668576},{-70314648,-102518775},{-70368367,-102484336},{-70581783,-102356424},{-70806711,-102250040},
        {-70871040,-102220919},{-71102823,-102137992},{-71139752,-102126095},{-71377383,-102066568},{-71417487,-102057104},{-71441248,-102053079},{-71682119,-102017352},{-71706535,-102014392},
        {-71731784,-102012543},{-71974456,-102000624},{-71999999,-102000000},{-104000000,-102000000},{-104025536,-102000624},{-104268207,-102012543},{-104293455,-102014392},
        {-104317880,-102017352},{-104558751,-102053079},{-104582512,-102057104},{-104620616,-102066008},{-104858200,-102125528},{-104894751,-102137111},{-105126455,-102220007},
        {-105161408,-102234136},{-105385056,-102339920},{-105418215,-102356415},{-105631632,-102484328},{-105676247,-102512143},{-105877839,-102661647},{-105917600,-102693352},
        {-106105199,-102863383},{-106121320,-102878680},{-106292007,-103066999},{-106331424,-103112656},{-106481224,-103314648},{-106515663,-103368367},{-106643575,-103581783},
        {-106749959,-103806711},{-106779080,-103871040},{-106862007,-104102823},{-106873904,-104139752},{-106933431,-104377383},{-106942896,-104417487},{-106946920,-104441248},
        {-106982648,-104682119},{-106985607,-104706535},{-106987456,-104731784},{-107000630,-105000000},{-112000000,-105000000},{-112294056,-105014448},{-112585264,-105057640},
        {-112870848,-105129184},{-112919359,-105146535},{-113148048,-105228360},{-113194624,-105250392},{-113414191,-105354239},{-113666711,-105505591},{-113708095,-105536279},
        {-113903183,-105680967},{-114121320,-105878679},{-114319032,-106096816},{-114349720,-106138200},{-114494408,-106333288},{-114645760,-106585808},{-114667792,-106632384},
        {-114771640,-106851952},{-114788991,-106900463},{-114870815,-107129151},{-114942359,-107414735},{-114985551,-107705943},{-115000000,-107999999},{-115000000,-134000000},
        {-114985551,-134294048},{-114942359,-134585263},{-114870816,-134870847},{-114853464,-134919359},{-114771639,-135148064},{-114645759,-135414192},{-114494407,-135666720},
        {-114319031,-135903184},{-114121320,-136121327},{-114083144,-136155919},{-113903184,-136319023},{-113861799,-136349712},{-113666711,-136494416},{-113458383,-136619264},
        {-113414192,-136645743},{-113148049,-136771631},{-112870848,-136870815},{-112820872,-136883327},{-112585264,-136942351},{-112534303,-136949920},{-112294056,-136985551},
        {-112000000,-137000000},{-107000630,-137000000},{-106987456,-137268207},{-106985608,-137293440},{-106982647,-137317872},{-106946920,-137558751},{-106942896,-137582511},
        {-106933991,-137620624},{-106874471,-137858208},{-106862888,-137894751},{-106779992,-138126463},{-106765863,-138161424},{-106660080,-138385055},{-106643584,-138418223},
        {-106515671,-138631648},{-106487855,-138676256},{-106338352,-138877839},{-106306647,-138917600},{-106136616,-139105199},{-106121320,-139121328},{-105933000,-139291999},
        {-105887344,-139331407},{-105685351,-139481232},{-105631632,-139515663},{-105418216,-139643567},{-105193288,-139749951},{-105128959,-139779072},{-104897175,-139862016},
        {-104860247,-139873904},{-104622616,-139933423},{-104582511,-139942896},{-104558751,-139946912},{-104317880,-139982656},{-104293463,-139985616},{-104268216,-139987456},
        {-104025544,-139999376},{-104000000,-140000000},{-71999999,-140000000}
    };
    expolygon.holes = {
        {{-105000000,-138000000},{-105000000,-104000000},{-71000000,-104000000},{-71000000,-138000000}},
        {{-69000000,-132000000},{-69000000,-110000000},{-64991180,-110000000},{-64991180,-132000000}},
        {{-111008824,-132000000},{-111008824,-110000000},{-107000000,-110000000},{-107000000,-132000000}}
    };
    PrintObject *object = print.get_object(0);
    object->slice();
    Layer       *layer = object->get_layer(1);
    LayerRegion *layerm = layer->get_region(0);
    layerm->m_slices.clear();
    layerm->m_slices.append({ expolygon }, stInternal);
    layer->lslices = { expolygon };
    layer->lslices_ex = { { get_extents(expolygon) } };
    
    // make perimeters
    layer->make_perimeters();
    
    // compute the covered area
    Flow pflow = layerm->flow(frPerimeter);
    Flow iflow = layerm->flow(frInfill);
    Polygons covered_by_perimeters;
    Polygons covered_by_infill;
    {
        Polygons acc;
        for (const ExtrusionEntity *ee : layerm->perimeters())
            for (const ExtrusionEntity *ee : dynamic_cast<const ExtrusionEntityCollection*>(ee)->entities)
                append(acc, offset(dynamic_cast<const ExtrusionLoop*>(ee)->polygon().split_at_first_point(), float(pflow.scaled_width() / 2.f + SCALED_EPSILON)));
        covered_by_perimeters = union_(acc);
    }
    {
        Polygons acc;
        for (const ExPolygon &expolygon : layerm->fill_expolygons())
            append(acc, to_polygons(expolygon));
        for (const ExtrusionEntity *ee : layerm->thin_fills().entities)
            append(acc, offset(dynamic_cast<const ExtrusionPath*>(ee)->polyline, float(iflow.scaled_width() / 2.f + SCALED_EPSILON)));
        covered_by_infill = union_(acc);
    }
    
    // compute the non covered area
    ExPolygons non_covered = diff_ex(to_polygons(layerm->slices().surfaces), union_(covered_by_perimeters, covered_by_infill));
    
    /*
    if (0) {
        printf "max non covered = %f\n", List::Util::max(map unscale unscale $_->area, @$non_covered);
        require "Slic3r/SVG.pm";
        Slic3r::SVG::output(
            "gaps.svg",
            expolygons          => [ map $_->expolygon, @{$layerm->slices} ],
            red_expolygons      => union_ex([ map @$_, (@$covered_by_perimeters, @$covered_by_infill) ]),
            green_expolygons    => union_ex($non_covered),
            no_arrows           => 1,
            polylines           => [
                map $_->polygon->split_at_first_point, map @$_, @{$layerm->perimeters},
            ],
        );
    }
    */
    THEN("no gap between perimeters and infill") {
        size_t num_non_convered = std::count_if(non_covered.begin(), non_covered.end(), 
            [&iflow](const ExPolygon &ex){ return ex.area() > sqr(double(iflow.scaled_width())); });
        REQUIRE(num_non_convered == 0);
    }
}

SCENARIO("Perimeters3", "[Perimeters]")
{
    auto config = Slic3r::DynamicPrintConfig::full_print_config_with({
        { "skirts",                 0 },
        { "perimeters",             3 },
        { "layer_height",           0.15 },
        { "bridge_speed",           99 },
        { "enable_dynamic_overhang_speeds",         false },
        // to prevent bridging over sparse infill
        { "fill_density",           0 },
        { "overhangs",              true },
        // to prevent speeds from being altered
        { "cooling",                "0" },
        // to prevent speeds from being altered
        { "first_layer_speed",      "100%" }
    });

    auto test = [&config](const Vec3d &scale) {
        std::string         gcode = Slic3r::Test::slice({ mesh(Slic3r::Test::TestMesh::V, Vec3d::Zero(), scale) }, config);
        GCodeReader         parser;
        std::set<coord_t>   z_with_bridges;
        const double        bridge_speed = config.opt_float("bridge_speed") * 60.;
        parser.parse_buffer(gcode, [&z_with_bridges, bridge_speed](Slic3r::GCodeReader &self, const Slic3r::GCodeReader::GCodeLine &line)
        {
            if (line.extruding(self) && line.dist_XY(self) > 0 && is_approx<double>(line.new_F(self), bridge_speed))
                z_with_bridges.insert(scaled<coord_t>(self.z()));
        });
        return z_with_bridges.size();
    };

    GIVEN("V shape, unscaled") {
        int n = test(Vec3d(1., 1., 1.));
        // One bridge layer under the V middle and one layer (two briding areas) under tops
        THEN("no overhangs printed with bridge speed") {
            REQUIRE(n == 2);
        }
    }
    GIVEN("V shape, scaled 3x in X") {
        int n = test(Vec3d(3., 1., 1.));
        // except for the two internal solid layers above void
        THEN("overhangs printed with bridge speed") {
            REQUIRE(n > 2);
        }
    }
}

SCENARIO("Perimeters4", "[Perimeters]")
{
    auto config = Slic3r::DynamicPrintConfig::full_print_config_with({
        { "seam_position",        "random" }
    });
    std::string gcode = Slic3r::Test::slice({ Slic3r::Test::TestMesh::cube_20x20x20 }, config);
    THEN("successful generation of G-code with seam_position = random") {
        REQUIRE(! gcode.empty());
    }
}

SCENARIO("Seam alignment", "[Perimeters]")
{
    auto test = [](Test::TestMesh model) {
        auto config = Slic3r::DynamicPrintConfig::full_print_config_with({
            { "seam_position",          "aligned" },
            { "skirts",                 0 },
            { "perimeters",             1 },
            { "fill_density",           0 },
            { "top_solid_layers",       0 },
            { "bottom_solid_layers",    0 },
            { "retract_layer_change",   "0" }
        });
        std::string gcode = Slic3r::Test::slice({ model }, config);
        bool        was_extruding = false;
        Points      seam_points;
        GCodeReader parser;
        parser.parse_buffer(gcode, [&was_extruding, &seam_points](Slic3r::GCodeReader &self, const Slic3r::GCodeReader::GCodeLine &line)
        {
            if (line.extruding(self)) {
                if (! was_extruding)
                    seam_points.emplace_back(self.xy_scaled());
                was_extruding = true;
            } else if (! line.cmd_is("M73")) {
                // skips remaining time lines (M73)
                was_extruding = false;
            }
        });
        THEN("seam is aligned") {
            size_t num_not_aligned = 0;
            for (size_t i = 1; i < seam_points.size(); ++ i) {
                double d = (seam_points[i] - seam_points[i - 1]).cast<double>().norm();
                // Seams shall be aligned up to 3mm.
                if (d > scaled<double>(3.))
                    ++ num_not_aligned;
            }
            REQUIRE(num_not_aligned == 0);
        }
    };

    GIVEN("20mm cube") {
        test(Slic3r::Test::TestMesh::cube_20x20x20);
    }
    GIVEN("small_dorito") {
        test(Slic3r::Test::TestMesh::small_dorito);
    }
}

// ---------------------------------------------------------------------------
// Fork-specific tests: new perimeter roles and print-order options (H-4)
// ---------------------------------------------------------------------------

SCENARIO("Fork: perimeter role assignment for new roles in classic mode", "[Perimeters]")
{
    // With perimeters=3 on a plain rectangle, process_classic must assign:
    //   depth 0  ->  ExternalPerimeter
    //   depth 1  ->  FirstInternalPerimeter   (fork-new role)
    //   depth 2  ->  SecondInternalPerimeter  (fork-new role)
    // Use layer_id=-1 so overhang detection is skipped and each loop
    // consists of a single ExtrusionPath whose role() is unambiguous.
    FullPrintConfig config;
    config.perimeters.value = 3;

    SurfaceCollection slices;
    slices.append(
        { ExPolygon{ Polygon::new_scale({ {0,0}, {100,0}, {100,100}, {0,100} }) } },
        stInternal);

    ExtrusionEntityCollection loops;
    ExtrusionEntityCollection gap_fill;
    ExPolygons                fill_expolygons;
    Flow                      flow(1., 1., 1.);
    PerimeterRegions          perimeter_regions;
    PerimeterGenerator::Parameters params(
        1.,  // layer height
        -1,  // layer ID — negative keeps overhang detection disabled
        flow, flow, flow, flow, flow, flow,
        static_cast<const PrintRegionConfig&>(config),
        static_cast<const PrintObjectConfig&>(config),
        static_cast<const PrintConfig&>(config),
        perimeter_regions,
        false); // spiral_vase
    Polygons lower_layer_polygons_cache;
    for (const Surface &surface : slices)
        PerimeterGenerator::process_classic(
            params,
            surface,
            nullptr,
            nullptr,
            lower_layer_polygons_cache,
            loops, gap_fill, fill_expolygons);

    loops = loops.flatten();

    THEN("3 loops generated for 3 perimeters") {
        REQUIRE(loops.entities.size() == 3);
    }
    THEN("exactly one ExternalPerimeter loop") {
        size_t n = std::count_if(loops.entities.begin(), loops.entities.end(),
            [](const ExtrusionEntity *ee){ return ee->role() == ExtrusionRole::ExternalPerimeter; });
        REQUIRE(n == 1);
    }
    THEN("exactly one FirstInternalPerimeter loop") {
        size_t n = std::count_if(loops.entities.begin(), loops.entities.end(),
            [](const ExtrusionEntity *ee){ return ee->role() == ExtrusionRole::FirstInternalPerimeter; });
        REQUIRE(n == 1);
    }
    THEN("exactly one SecondInternalPerimeter loop") {
        size_t n = std::count_if(loops.entities.begin(), loops.entities.end(),
            [](const ExtrusionEntity *ee){ return ee->role() == ExtrusionRole::SecondInternalPerimeter; });
        REQUIRE(n == 1);
    }
}

SCENARIO("Fork: reverse_internal_perimeters smoke test", "[Perimeters]")
{
    // Smoke test: reverse_internal_perimeters must slice a simple cube without
    // crashing.  Non-empty G-code is the primary correctness criterion.
    auto config = Slic3r::DynamicPrintConfig::full_print_config_with({
        { "skirts",                      0 },
        { "perimeters",                  3 },
        { "reverse_internal_perimeters", true },
        { "cooling",                     "0" },
        { "first_layer_speed",           "100%" }
    });
    std::string gcode = Slic3r::Test::slice({ Slic3r::Test::TestMesh::cube_20x20x20 }, config);
    THEN("G-code produced without crash") {
        REQUIRE(! gcode.empty());
    }
}

// ---------------------------------------------------------------------------
// Fork: combine_perimeters (per-loop rework) tests.
// ---------------------------------------------------------------------------

// Square frustum (truncated pyramid) narrowing upward with planar side walls. The
// perimeter centerlines are parallel to the walls, so between adjacent layers they
// shift horizontally by exactly layer_height * (base_half - top_half) / height.
static Slic3r::TriangleMesh make_frustum(float base_half, float top_half, float height)
{
    const float b = base_half, t = top_half, h = height;
    return Slic3r::TriangleMesh(
        { {-b,-b,0.f}, {-b,b,0.f}, {b,b,0.f}, {b,-b,0.f},
          {-t,-t,h},   {-t,t,h},   {t,t,h},   {t,-t,h} },
        { {0,2,3}, {0,1,3},             // bottom, outward -Z
          {4,6,5}, {4,7,6},             // top, outward +Z
          {0,5,1}, {0,4,5},             // x = -b wall
          {1,6,2}, {1,5,6},             // y = +b wall
          {2,7,3}, {2,6,7},             // x = +b wall
          {3,4,0}, {3,7,4} });          // y = -b wall
}


namespace {

constexpr double kStadium = 1. - M_PI / 4.;
const char *const kLoopMethods[] = { "loop_strict", "loop_tolerant", "arc_coverage", "legacy_mask_v3" };

using Overrides = std::vector<std::pair<std::string, std::string>>;

// Base configuration of the combine tests: layer height lh, N layers per combine group for both
// internal roles, perimeter width 0.45, no skirt, cooling off, light infill.
DynamicPrintConfig cp_config(const std::string &method, double lh, int n, const std::string &shift = "20%",
                             const std::string &generator = "classic", const Overrides &extra = {})
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config_with({
        { "skirts",                                 0 },
        { "perimeters",                             3 },
        { "layer_height",                           lh },
        { "first_layer_height",                     lh },
        { "perimeter_extrusion_width",              0.45 },
        { "external_perimeter_extrusion_width",     0.45 },
        { "perimeter_generator",                    generator },
        { "first_internal_perimeter_every_layers",  n },
        { "second_internal_perimeter_every_layers", n },
        { "combine_perimeters_method",              method },
        { "combine_perimeters_max_shift",           shift },
        { "fill_density",                           "5%" },
        { "cooling",                                "0" },
        { "first_layer_speed",                      "100%" }
    });
    for (const auto &kv : extra)
        cfg.set_deserialize_strict(kv.first, kv.second);
    return cfg;
}

struct Sliced {
    Model model;
    Print print;
};

std::unique_ptr<Sliced> cp_slice(const TriangleMesh &mesh, const DynamicPrintConfig &cfg)
{
    auto s = std::make_unique<Sliced>();
    Test::init_print(std::vector<TriangleMesh>{ mesh }, s->print, s->model, cfg);
    s->print.set_status_silent();
    s->print.process();
    return s;
}

TriangleMesh box_mesh(double sx, double sy, double sz, double x, double y, double z)
{
    TriangleMesh m = make_cube(sx, sy, sz);
    m.translate(float(x - 0.5 * sx), float(y - 0.5 * sy), float(z));
    return m;
}

// One extruded entity of a layer: a loop, an open multipath or a single path.
struct EntityInfo {
    const ExtrusionEntity               *entity;
    std::vector<const ExtrusionPath *>   paths;
    bool                                 closed;
};

void collect_entities(const ExtrusionEntity *e, std::vector<EntityInfo> &out)
{
    if (e->is_collection()) {
        for (const ExtrusionEntity *c : static_cast<const ExtrusionEntityCollection*>(e)->entities)
            collect_entities(c, out);
    } else if (auto *loop = dynamic_cast<const ExtrusionLoop*>(e)) {
        EntityInfo info{ e, {}, true };
        for (const ExtrusionPath &p : loop->paths)
            info.paths.push_back(&p);
        out.emplace_back(std::move(info));
    } else if (auto *mp = dynamic_cast<const ExtrusionMultiPath*>(e)) {
        EntityInfo info{ e, {}, false };
        for (const ExtrusionPath &p : mp->paths)
            info.paths.push_back(&p);
        out.emplace_back(std::move(info));
    } else if (auto *p = dynamic_cast<const ExtrusionPath*>(e)) {
        out.push_back({ e, { p }, false });
    }
}

std::vector<EntityInfo> layer_entities(const Print &print, size_t layer_idx)
{
    std::vector<EntityInfo> out;
    for (const LayerRegion *lr : print.objects().front()->layers()[layer_idx]->regions())
        collect_entities(&lr->perimeters(), out);
    return out;
}

double layer_height_of(const Print &print, size_t layer_idx) { return print.objects().front()->layers()[layer_idx]->height; }
size_t layer_count(const Print &print) { return print.objects().front()->layers().size(); }

bool is_thick(const ExtrusionPath &p, double lh) { return p.height() > lh + 1e-3; }

bool is_internal_role(const ExtrusionPath &p)
{
    return p.role() == ExtrusionRole::FirstInternalPerimeter || p.role() == ExtrusionRole::SecondInternalPerimeter;
}

size_t count_thick_paths(const Print &print)
{
    size_t n = 0;
    for (size_t i = 0; i < layer_count(print); ++ i)
        for (const EntityInfo &e : layer_entities(print, i))
            for (const ExtrusionPath *p : e.paths)
                if (is_internal_role(*p) && is_thick(*p, layer_height_of(print, i)))
                    ++ n;
    return n;
}

using Seg  = std::pair<Vec2d, Vec2d>;
using Segs = std::vector<Seg>;

Vec2d to_mm(const Point &p) { return Vec2d(unscale<double>(p.x()), unscale<double>(p.y())); }

void path_segments(const ExtrusionPath &p, Segs &out)
{
    for (size_t i = 1; i < p.polyline.points.size(); ++ i)
        out.emplace_back(to_mm(p.polyline.points[i - 1]), to_mm(p.polyline.points[i]));
}

void path_samples(const ExtrusionPath &p, double step, std::vector<Vec2d> &out)
{
    double carry = 0.;
    for (size_t i = 1; i < p.polyline.points.size(); ++ i) {
        const Vec2d a = to_mm(p.polyline.points[i - 1]), b = to_mm(p.polyline.points[i]);
        const double len = (b - a).norm();
        double t = carry;
        while (t < len) {
            out.push_back(a + (b - a) * (t / std::max(len, 1e-12)));
            t += step;
        }
        carry = t - len;
    }
}

double dist_to_segs(const Vec2d &p, const Segs &segs)
{
    double best = std::numeric_limits<double>::max();
    for (const Seg &s : segs) {
        const Vec2d d = s.second - s.first;
        const double l2 = d.squaredNorm();
        double t = l2 > 0. ? (p - s.first).dot(d) / l2 : 0.;
        t = std::clamp(t, 0., 1.);
        best = std::min(best, (p - (s.first + d * t)).norm());
    }
    return best;
}

Segs role_segments(const Print &print, size_t layer_idx, ExtrusionRole role)
{
    Segs segs;
    for (const EntityInfo &e : layer_entities(print, layer_idx))
        for (const ExtrusionPath *p : e.paths)
            if (p->role() == role)
                path_segments(*p, segs);
    return segs;
}

struct Protection {
    size_t lost_samples     { 0 };  // base samples of a role that the combined run no longer prints on the layer
    size_t lost_exposed     { 0 };  // ... with no counterpart on the (N-1) layers above: P1 violation
    size_t thick_samples    { 0 };
    size_t thick_unsupported{ 0 };  // thick samples without a counterpart on a layer below: P2 violation
};

// Compares a combined run with the same model sliced without combining (every-N = 1).
Protection cp_protection(const Print &comb, const Print &base, int n_window, double tol)
{
    Protection out;
    REQUIRE(layer_count(comb) == layer_count(base));
    const size_t nl = layer_count(base);
    for (ExtrusionRole role : { ExtrusionRole::FirstInternalPerimeter, ExtrusionRole::SecondInternalPerimeter }) {
        std::vector<Segs> base_segs(nl);
        for (size_t i = 0; i < nl; ++ i)
            base_segs[i] = role_segments(base, i, role);
        for (size_t i = 0; i < nl; ++ i) {
            const Segs comb_segs = role_segments(comb, i, role);
            std::vector<Vec2d> samples;
            for (const EntityInfo &e : layer_entities(base, i))
                for (const ExtrusionPath *p : e.paths)
                    if (p->role() == role)
                        path_samples(*p, 0.5, samples);
            for (const Vec2d &s : samples) {
                if (! comb_segs.empty() && dist_to_segs(s, comb_segs) <= 0.02)
                    continue;
                ++ out.lost_samples;
                bool ok = false;
                for (size_t j = i + 1; j < nl && j < i + size_t(n_window) && ! ok; ++ j)
                    ok = ! base_segs[j].empty() && dist_to_segs(s, base_segs[j]) <= tol;
                if (! ok)
                    ++ out.lost_exposed;
            }
            const double lh = layer_height_of(comb, i);
            for (const EntityInfo &e : layer_entities(comb, i))
                for (const ExtrusionPath *p : e.paths) {
                    if (p->role() != role || ! is_thick(*p, lh))
                        continue;
                    const int k = int(std::lround(p->height() / lh));
                    std::vector<Vec2d> ts;
                    path_samples(*p, 0.5, ts);
                    for (const Vec2d &s : ts) {
                        ++ out.thick_samples;
                        bool ok = true;
                        for (int m = 1; m < k && ok; ++ m)
                            ok = size_t(m) <= i && ! base_segs[i - m].empty() && dist_to_segs(s, base_segs[i - m]) <= tol;
                        if (! ok)
                            ++ out.thick_unsupported;
                    }
                }
        }
    }
    return out;
}

// (type, height in microns) of every extruding move of the G-code.
std::set<std::pair<std::string, int>> gcode_type_heights(const std::string &gcode)
{
    std::set<std::pair<std::string, int>> out;
    std::istringstream ss(gcode);
    std::string line, type;
    double height = 0.;
    while (std::getline(ss, line)) {
        if (line.compare(0, 6, ";TYPE:") == 0)
            type = line.substr(6);
        else if (line.compare(0, 8, ";HEIGHT:") == 0)
            height = std::stod(line.substr(8));
        else if (line.compare(0, 2, "G1") == 0) {
            const size_t e = line.find(" E");
            if (e != std::string::npos && std::stod(line.substr(e + 2)) > 0.)
                out.insert({ type, int(std::lround(height * 1000.)) });
        }
    }
    return out;
}

bool gcode_has_internal_height(const std::string &gcode, int height_um, const char *type = nullptr)
{
    for (const auto &th : gcode_type_heights(gcode))
        if (th.second == height_um && (type ? th.first == type : (th.first == "First internal perimeter" || th.first == "Second internal perimeter")))
            return true;
    return false;
}

bool gcode_has_internal_thicker_than(const std::string &gcode, int height_um, const char *type)
{
    for (const auto &th : gcode_type_heights(gcode))
        if (th.first == type && th.second > height_um)
            return true;
    return false;
}

} // namespace

SCENARIO("Fork: combine_perimeters_max_shift displacement matrix", "[Perimeters]")
{
    // Layer height 0.15, N = 2 (groups of 0.3 under the 0.4 nozzle cap), perimeter width 0.45.
    // New semantics: the shift is the largest lateral displacement of a loop between adjacent layers
    // (measured along the normal, so box corners do not add a sqrt(2) artefact).
    //  - shallow frustum: walls shift 0.08 mm per layer = 0.18 w
    //  - steep frustum:   walls shift 0.16 mm per layer = 0.36 w
    const TriangleMesh shallow = make_frustum(10.f, 6.8f, 6.f);
    const TriangleMesh steep   = make_frustum(10.f, 3.6f, 6.f);
    const TriangleMesh cube    = box_mesh(20, 20, 6, 0, 0, 0);

    for (const char *method : kLoopMethods) {
        if (std::string(method) == "legacy_mask_v3")
            continue; // mask based: own expectations below
        CAPTURE(method);
        auto combined = [&](const TriangleMesh &mesh, const char *shift) {
            return gcode_has_internal_height(Test::slice({ mesh }, cp_config(method, 0.15, 2, shift)), 300);
        };
        // Perfectly vertical walls: combine at 20% and 100%, and also at shift 0 (coincident loops).
        CHECK(combined(cube, "20%"));
        CHECK(combined(cube, "100%"));
        CHECK(combined(cube, "0%"));
        // Walls shifting 0.18 w per layer: combine at 20%, 50% and 100%, not at 10%.
        CHECK(combined(shallow, "20%"));
        CHECK(combined(shallow, "50%"));
        CHECK(combined(shallow, "100%"));
        CHECK(! combined(shallow, "10%"));
        // Walls shifting 0.36 w per layer: not at 20%; at 40%, 50% and 100% they combine, the box
        // corner artefact (corner shift sqrt(2) x wall shift) must not block combining.
        CHECK(! combined(steep, "20%"));
        CHECK(combined(steep, "40%"));
        CHECK(combined(steep, "50%"));
        CHECK(combined(steep, "100%"));
    }

    // legacy_mask_v3 is mask based: the shift is the mask radius, and (with the legacy tolerance of a
    // total uncovered length of half a width) the diagonal displacement of a box corner (sqrt(2) x the
    // wall shift) is not forgiven, so it needs a larger shift than the normal-distance loop methods.
    {
        auto combined = [&](const TriangleMesh &mesh, const char *shift) {
            return gcode_has_internal_height(Test::slice({ mesh }, cp_config("legacy_mask_v3", 0.15, 2, shift)), 300);
        };
        CHECK(combined(cube, "20%"));
        CHECK(combined(cube, "100%"));
        CHECK(combined(cube, "0%"));
        CHECK(combined(shallow, "50%"));
        CHECK(combined(shallow, "100%"));
        CHECK(! combined(shallow, "10%"));
        CHECK(! combined(steep, "20%"));
        CHECK(combined(steep, "100%"));
    }
}

SCENARIO("Fork: combine_perimeters all methods are selectable", "[Perimeters]")
{
    for (const char *method : { "legacy_mask", "legacy_mask_v2", "legacy_mask_v3", "loop_strict", "loop_tolerant", "arc_coverage" }) {
        CAPTURE(method);
        const std::string gcode = Test::slice({ box_mesh(20, 20, 6, 0, 0, 0) }, cp_config(method, 0.15, 2));
        REQUIRE(! gcode.empty());
        REQUIRE(gcode_has_internal_height(gcode, 300));
    }
}

SCENARIO("Fork: combine_perimeters protection P1 exposed sub-layer loop under a top surface", "[Perimeters]")
{
    // A 20x20x3 block with a 10x10x6 block on top: the wide block's perimeters in the ring outside the
    // narrow block have nothing above them (they sit under a top surface) and must never be voided.
    TriangleMesh stepped = box_mesh(20, 20, 3.3, 0, 0, 0);
    stepped.merge(box_mesh(10, 10, 6, 0, 0, 3.0));
    const auto base = cp_slice(stepped, cp_config("loop_tolerant", 0.15, 1));
    for (const char *method : kLoopMethods) {
        CAPTURE(method);
        const auto comb = cp_slice(stepped, cp_config(method, 0.15, 2));
        const Protection p = cp_protection(comb->print, base->print, 2, 0.11);
        CHECK(count_thick_paths(comb->print) > 0);
        CHECK(p.lost_samples > 0);
        CHECK(p.lost_exposed == 0);
        CHECK(p.thick_unsupported == 0);
    }
}

SCENARIO("Fork: combine_perimeters protection P2 widening wall combines nothing", "[Perimeters]")
{
    // Inverted steep frustum: every loop of a layer is 0.36 w outside of the layer below.
    const TriangleMesh inverted = make_frustum(3.6f, 10.f, 6.f);
    for (const char *method : kLoopMethods) {
        CAPTURE(method);
        const auto comb = cp_slice(inverted, cp_config(method, 0.15, 2));
        REQUIRE(count_thick_paths(comb->print) == 0);
    }
    // Same for the shallow widening wall: if anything combines it must be supported from below.
    const TriangleMesh inverted_shallow = make_frustum(6.8f, 10.f, 6.f);
    const auto base = cp_slice(inverted_shallow, cp_config("loop_tolerant", 0.15, 1));
    for (const char *method : kLoopMethods) {
        CAPTURE(method);
        const auto comb = cp_slice(inverted_shallow, cp_config(method, 0.15, 2));
        // Point distance to a polyline is sqrt(2) x the normal shift at box corners: tolerance 0.13.
        const Protection p = cp_protection(comb->print, base->print, 2, 0.13);
        CHECK(p.lost_exposed == 0);
        CHECK(p.thick_unsupported == 0);
    }
}

SCENARIO("Fork: combine_perimeters protection P3 vanishing inner loop is never voided", "[Perimeters]")
{
    // A pointed taper: 6 mm half width to 0.4 mm over 24 mm. Towards the tip the part loses its second
    // and then its first internal loop; the vanishing loops have no counterpart above.
    const TriangleMesh taper = make_frustum(6.f, 0.4f, 24.f);
    const auto base = cp_slice(taper, cp_config("loop_tolerant", 0.15, 1));
    for (const char *method : kLoopMethods) {
        CAPTURE(method);
        const auto comb = cp_slice(taper, cp_config(method, 0.15, 2));
        const Protection p = cp_protection(comb->print, base->print, 2, 0.11);
        CHECK(count_thick_paths(comb->print) > 0);
        CHECK(p.lost_exposed == 0);
        CHECK(p.thick_unsupported == 0);
    }
}

SCENARIO("Fork: combine_perimeters protection P4 hole starting mid-window", "[Perimeters]")
{
    // A 30x30 plate whose 14x14 hole starts at z = hole_z: four boxes around the hole sit on a base.
    for (double hole_z : { 1.0, 1.15 }) {
        CAPTURE(hole_z);
        auto plate = [&](double /*unused*/) {
            TriangleMesh m = box_mesh(30, 30, hole_z + 0.1, 0, 0, 0);
            m.merge(box_mesh(30, 8, 3.3, 0, -11, hole_z));
            m.merge(box_mesh(30, 8, 3.3, 0,  11, hole_z));
            m.merge(box_mesh(8, 14, 3.3, -11, 0, hole_z));
            m.merge(box_mesh(8, 14, 3.3,  11, 0, hole_z));
            return m;
        }(0.);
        for (const char *method : kLoopMethods) {
            CAPTURE(method);
            const auto comb = cp_slice(plate, cp_config(method, 0.15, 2));
            // The outer loops still combine.
            bool outer_combined = false;
            // First layer that has a hole loop (a loop whose bbox lies inside the 14 mm hole region).
            size_t first_hole_layer = size_t(-1);
            for (size_t i = 0; i < layer_count(comb->print); ++ i) {
                const double lh = layer_height_of(comb->print, i);
                for (const EntityInfo &e : layer_entities(comb->print, i)) {
                    BoundingBox bb;
                    for (const ExtrusionPath *p : e.paths)
                        bb.merge(Points(p->polyline.points));
                    const bool is_hole_loop = bb.defined && unscale<double>(bb.size().x()) < 20. && unscale<double>(bb.size().x()) > 5.;
                    bool thick = false;
                    for (const ExtrusionPath *p : e.paths)
                        thick |= is_internal_role(*p) && is_thick(*p, lh);
                    if (is_hole_loop) {
                        if (first_hole_layer == size_t(-1))
                            first_hole_layer = i;
                        // Hole loops on their first layer sit over solid material only: never thickened.
                        if (i == first_hole_layer)
                            CHECK(! thick);
                    } else
                        outer_combined |= thick;
                }
            }
            CHECK(first_hole_layer != size_t(-1));
            CHECK(outer_combined);
        }
    }
}

SCENARIO("Fork: combine_perimeters protection P5 bridge perimeters are untouched", "[Perimeters]")
{
    // A 30 mm slab on two pillars leaves a 20 mm bridge: the slab's first layers have overhang
    // (bridge-role) perimeter paths in loops that also contain ordinary paths over the pillars.
    TriangleMesh arch = box_mesh(6, 14, 4.4, -12, 0, 0);
    arch.merge(box_mesh(6, 14, 4.4, 12, 0, 0));
    arch.merge(box_mesh(30, 14, 3.3, 0, 0, 4.2));
    const auto base = cp_slice(arch, cp_config("loop_tolerant", 0.15, 1, "20%", "classic", { { "overhangs", "1" } }));

    struct Sig { size_t paths = 0, mixed_loops = 0; double mm3 = 0., max_height = 0.; };
    auto signature = [](const Print &print) {
        Sig sig;
        for (size_t i = 0; i < layer_count(print); ++ i)
            for (const EntityInfo &e : layer_entities(print, i)) {
                size_t bridge = 0;
                for (const ExtrusionPath *p : e.paths)
                    if (p->role().is_bridge() && p->role().is_perimeter()) {
                        ++ bridge;
                        ++ sig.paths;
                        sig.mm3 += p->mm3_per_mm() * unscale<double>(p->length());
                        sig.max_height = std::max<double>(sig.max_height, p->height());
                    }
                if (bridge > 0 && bridge < e.paths.size())
                    ++ sig.mixed_loops;
            }
        return sig;
    };
    const Sig base_sig = signature(base->print);
    REQUIRE(base_sig.paths > 0);

    for (const char *method : kLoopMethods) {
        CAPTURE(method);
        const auto comb = cp_slice(arch, cp_config(method, 0.15, 2, "20%", "classic", { { "overhangs", "1" } }));
        const Sig sig = signature(comb->print);
        // Bridge perimeter paths are never voided or thickened.
        CHECK(sig.paths == base_sig.paths);
        CHECK(sig.mm3 == Approx(base_sig.mm3).epsilon(1e-6));
        CHECK(sig.max_height == Approx(base_sig.max_height).epsilon(1e-6));
        // P4/f: mixed bridge/non-bridge loops are not half-voided, and no path of such a loop is thick.
        if (std::string(method) != "arc_coverage") {
            CHECK(sig.mixed_loops == base_sig.mixed_loops);
            for (size_t i = 0; i < layer_count(comb->print); ++ i)
                for (const EntityInfo &e : layer_entities(comb->print, i)) {
                    bool has_bridge = false, has_thick = false;
                    for (const ExtrusionPath *p : e.paths) {
                        has_bridge |= p->role().is_bridge();
                        has_thick  |= is_thick(*p, layer_height_of(comb->print, i)) && ! p->role().is_bridge();
                    }
                    CHECK(! (has_bridge && has_thick));
                }
        }
    }
}

SCENARIO("Fork: combine_perimeters whole-loop rule", "[Perimeters]")
{
    // Loop methods and legacy_mask_v3 never split a loop: all paths of one entity have the same height.
    const TriangleMesh shallow = make_frustum(10.f, 6.8f, 6.f);
    const TriangleMesh taper   = make_frustum(8.f, 1.f, 12.f);
    for (const char *generator : { "classic", "arachne" })
        for (const char *method : { "loop_strict", "loop_tolerant", "legacy_mask_v3" })
            for (const TriangleMesh *mesh : { &shallow, &taper }) {
                CAPTURE(generator, method);
                const std::string shift = std::string(method) == "legacy_mask_v3" ? "50%" : "20%"; // see the displacement matrix
                const auto s = cp_slice(*mesh, cp_config(method, 0.15, 2, shift, generator));
                size_t thick = 0;
                for (size_t i = 0; i < layer_count(s->print); ++ i)
                    for (const EntityInfo &e : layer_entities(s->print, i)) {
                        float h0 = -1.f;
                        for (const ExtrusionPath *p : e.paths) {
                            if (! is_internal_role(*p))
                                continue;
                            if (h0 < 0.f)
                                h0 = p->height();
                            CHECK(p->height() == Approx(h0).margin(1e-4));
                            thick += is_thick(*p, layer_height_of(s->print, i));
                        }
                    }
                CHECK(thick > 0);
            }
}

SCENARIO("Fork: combine_perimeters arc_coverage pieces and bookkeeping", "[Perimeters]")
{
    // A sphere: group sizes change with latitude, so arc_coverage produces partial loops.
    TriangleMesh sphere(its_make_sphere(6., PI / 16.));
    sphere.translate(0.f, 0.f, 6.f);
    const double min_arc = 5.;
    const auto base = cp_slice(sphere, cp_config("loop_tolerant", 0.15, 1));
    const auto s    = cp_slice(sphere, cp_config("arc_coverage", 0.15, 2, "20%", "classic", { { "combine_perimeters_min_arc", "5" } }));
    CHECK(count_thick_paths(s->print) > 0);

    for (size_t i = 0; i < layer_count(s->print); ++ i) {
        const double lh = layer_height_of(s->print, i);
        for (const EntityInfo &e : layer_entities(s->print, i)) {
            // Maximal runs of thick internal paths in an entity.
            const size_t n = e.paths.size();
            size_t thick_count = 0;
            for (const ExtrusionPath *p : e.paths)
                thick_count += is_internal_role(*p) && is_thick(*p, lh);
            if (thick_count == 0 || thick_count == n)
                continue;
            // Start at a non-thick path so closed loops do not wrap around a run.
            size_t start = 0;
            while (is_thick(*e.paths[start], lh) && is_internal_role(*e.paths[start]))
                ++ start;
            double run = 0.;
            for (size_t k = 1; k <= n; ++ k) {
                const ExtrusionPath &p = *e.paths[(start + k) % n];
                if (is_internal_role(p) && is_thick(p, lh))
                    run += unscale<double>(p.length());
                else {
                    if (run > 0.)
                        CHECK(run >= min_arc - 1e-3);
                    run = 0.;
                }
            }
            if (run > 0.)
                CHECK(run >= min_arc - 1e-3);
        }
    }
    // Entity bookkeeping: a layer that lost entities stores the pre-combine entity count.
    for (size_t i = 0; i < layer_count(s->print); ++ i) {
        size_t base_entities = 0, now_entities = 0;
        for (const LayerRegion *lr : base->print.objects().front()->layers()[i]->regions())
            for (const ExtrusionEntity *ee : lr->perimeters().entities)
                base_entities += ee->is_collection() ? static_cast<const ExtrusionEntityCollection*>(ee)->entities.size() : 1;
        int pre = -1;
        for (const LayerRegion *lr : s->print.objects().front()->layers()[i]->regions()) {
            pre = std::max(pre, lr->perimeter_entity_count_pre_combine());
            for (const ExtrusionEntity *ee : lr->perimeters().entities)
                now_entities += ee->is_collection() ? static_cast<const ExtrusionEntityCollection*>(ee)->entities.size() : 1;
        }
        if (now_entities < base_entities) {
            CHECK(pre >= 0);
            CHECK(size_t(pre) == base_entities);
        }
        // A stashed count always equals the entity count the layer had before combining (the layer
        // may later hold MORE entities than that: arc_coverage splits loops into pieces).
        if (pre >= 0)
            CHECK(size_t(pre) == base_entities);
    }
}

SCENARIO("Fork: combine_perimeters flow of combined paths", "[Perimeters][Flow]")
{
    // Volume conservation per path: mm3_per_mm == H * s_p with s_p the spacing of the thin path, the
    // combined width is at least its height, and per-path widths of Arachne walls are preserved.
    const TriangleMesh taper = make_frustum(8.f, 1.f, 12.f);
    for (const char *generator : { "classic", "arachne" })
        for (const char *method : { "loop_strict", "loop_tolerant", "legacy_mask_v3" }) {
            CAPTURE(generator, method);
            const std::string shift = std::string(method) == "legacy_mask_v3" ? "50%" : "20%";
            const auto base = cp_slice(taper, cp_config(method, 0.15, 1, shift, generator));
            const auto comb = cp_slice(taper, cp_config(method, 0.15, 2, shift, generator));
            REQUIRE(layer_count(base->print) == layer_count(comb->print));
            size_t thick = 0;
            std::set<int> thin_widths;
            for (size_t i = 0; i < layer_count(comb->print); ++ i) {
                const double lh = layer_height_of(comb->print, i);
                const auto ce = layer_entities(comb->print, i);
                const auto be = layer_entities(base->print, i);
                bool layer_has_thick = false;
                for (const EntityInfo &e : ce)
                    for (const ExtrusionPath *p : e.paths)
                        layer_has_thick |= is_thick(*p, lh);
                for (const EntityInfo &e : be)
                    for (const ExtrusionPath *p : e.paths)
                        thin_widths.insert(int(std::lround(p->width() * 1000.)));
                if (! layer_has_thick)
                    continue;
                // A top layer keeps the structure of an uncombined layer.
                REQUIRE(ce.size() == be.size());
                for (size_t k = 0; k < ce.size(); ++ k) {
                    REQUIRE(ce[k].paths.size() == be[k].paths.size());
                    for (size_t m = 0; m < ce[k].paths.size(); ++ m) {
                        const ExtrusionPath &c = *ce[k].paths[m], &b = *be[k].paths[m];
                        if (! is_thick(c, lh)) {
                            CHECK(c.width() == Approx(b.width()).margin(1e-5));
                            continue;
                        }
                        ++ thick;
                        const double H = c.height();
                        // Volume conservation: area = H * s_p, s_p of the thin path.
                        const double s_p = double(b.width()) - double(b.height()) * kStadium;
                        CHECK(c.mm3_per_mm() == Approx(H * s_p).margin(1e-6));
                        CHECK(c.mm3_per_mm() == Approx(H * (double(c.width()) - H * kStadium)).margin(1e-6));
                        CHECK(c.width() >= c.height());
                        // The path keeps its own width (plus the merge growth), not a loop-wide one.
                        CHECK(c.width() == Approx(b.width() + (H - b.height()) * kStadium).margin(1e-5));
                    }
                }
            }
            CHECK(thick > 0);
            if (std::string(generator) == "arachne")
                // The Arachne taper does produce paths of different widths.
                CHECK(thin_widths.size() > 1);
        }
}

SCENARIO("Fork: combine_perimeters legacy_mask_v2 flow", "[Perimeters][Flow]")
{
    // legacy_mask_v2 has the legacy acceptance test but the per-path, volume-conserving flow;
    // legacy_mask overwrites every combined path with the nominal width.
    const TriangleMesh taper = make_frustum(8.f, 1.f, 12.f);
    for (const char *generator : { "classic", "arachne" }) {
        CAPTURE(generator);
        const auto base = cp_slice(taper, cp_config("legacy_mask_v2", 0.15, 1, "50%", generator));
        const auto v1   = cp_slice(taper, cp_config("legacy_mask",    0.15, 2, "50%", generator));
        const auto v2   = cp_slice(taper, cp_config("legacy_mask_v2", 0.15, 2, "50%", generator));
        REQUIRE(layer_count(base->print) == layer_count(v2->print));
        REQUIRE(layer_count(v1->print) == layer_count(v2->print));
        size_t thick = 0;
        std::set<int> v1_widths, v2_widths;
        for (size_t i = 0; i < layer_count(v2->print); ++ i) {
            const double lh = layer_height_of(v2->print, i);
            const auto e1 = layer_entities(v1->print, i);
            const auto e2 = layer_entities(v2->print, i);
            const auto be = layer_entities(base->print, i);
            // Same accepted set: the same number of combined paths and of entities per layer.
            size_t n1 = 0, n2 = 0;
            for (const EntityInfo &e : e1)
                for (const ExtrusionPath *p : e.paths)
                    if (is_thick(*p, lh)) { ++ n1; v1_widths.insert(int(std::lround(p->width() * 1000.))); }
            for (const EntityInfo &e : e2)
                for (const ExtrusionPath *p : e.paths)
                    if (is_thick(*p, lh)) { ++ n2; v2_widths.insert(int(std::lround(p->width() * 1000.))); }
            CHECK(n1 == n2);
            REQUIRE(e1.size() == e2.size());
            if (n2 == 0)
                continue;
            REQUIRE(e2.size() == be.size());
            for (size_t k = 0; k < e2.size(); ++ k) {
                REQUIRE(e2[k].paths.size() == be[k].paths.size());
                for (size_t m = 0; m < e2[k].paths.size(); ++ m) {
                    const ExtrusionPath &c = *e2[k].paths[m], &b = *be[k].paths[m];
                    if (! is_thick(c, lh))
                        continue;
                    ++ thick;
                    const double H = c.height();
                    const double s_p = double(b.width()) - double(b.height()) * kStadium;
                    CHECK(c.mm3_per_mm() == Approx(H * s_p).margin(1e-6));
                    CHECK(c.width() >= c.height());
                }
            }
        }
        CHECK(thick > 0);
        // legacy_mask: every combined path shares the nominal width; v2 keeps per-path widths.
        CHECK(v1_widths.size() == 1);
        if (std::string(generator) == "arachne")
            CHECK(v2_widths.size() > 1);
    }
}

SCENARIO("Fork: combine_perimeters narrow perimeter width cap", "[Perimeters][Flow]")
{
    // w = 0.30, h = 0.1, N = 4: the merged bead (H = 0.4) would be narrower than tall. Nothing
    // may combine and nothing may throw.
    const TriangleMesh cube = box_mesh(20, 20, 4, 0, 0, 0);
    for (const char *method : kLoopMethods) {
        CAPTURE(method);
        auto cfg = cp_config(method, 0.1, 4, "20%", "classic",
            { { "perimeter_extrusion_width", "0.30" }, { "external_perimeter_extrusion_width", "0.30" } });
        std::unique_ptr<Sliced> s;
        REQUIRE_NOTHROW(s = cp_slice(cube, cfg));
        for (size_t i = 0; i < layer_count(s->print); ++ i)
            for (const EntityInfo &e : layer_entities(s->print, i))
                for (const ExtrusionPath *p : e.paths) {
                    CHECK(p->height() < 0.4f - 1e-3f);
                    CHECK(p->width() >= p->height());
                }
    }
}

SCENARIO("Fork: combine_perimeters config and legacy profiles", "[Perimeters][Config]")
{
    auto load = [](const std::string &ini) {
        DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
        REQUIRE_NOTHROW(cfg.load_from_ini_string(ini, ForwardCompatibilitySubstitutionRule::Disable));
        return cfg;
    };
    GIVEN("old overlap percentage") {
        const DynamicPrintConfig cfg = load("combine_perimeters_overlap_percent = 75\n");
        REQUIRE(! cfg.has("combine_perimeters_overlap_percent"));
        const auto *shift = cfg.option<ConfigOptionFloatOrPercent>("combine_perimeters_max_shift");
        REQUIRE(shift != nullptr);
        CHECK(shift->percent);
        CHECK(shift->value == Approx(20.));
    }
    GIVEN("old automatic key") {
        const DynamicPrintConfig on  = load("automatic_perimeter_combination = 1\n");
        const DynamicPrintConfig off = load("automatic_perimeter_combination = 0\n");
        CHECK(! on.has("automatic_perimeter_combination"));
        CHECK(on.opt_bool("automatic_internal_perimeters_combination"));
        CHECK(! off.opt_bool("automatic_internal_perimeters_combination"));
    }
    GIVEN("removed external_perimeter_every_layers") {
        const DynamicPrintConfig cfg = load("external_perimeter_every_layers = 2\nperimeters = 4\n");
        CHECK(! cfg.has("external_perimeter_every_layers"));
        CHECK(cfg.opt_int("perimeters") == 4);
    }
    GIVEN("the method enum") {
        CHECK(FullPrintConfig().combine_perimeters_method.value == cpmLoopTolerant);
        CHECK(DynamicPrintConfig::full_print_config().opt_enum<CombinePerimetersMethod>("combine_perimeters_method") == cpmLoopTolerant);
        const std::pair<const char *, CombinePerimetersMethod> values[] = {
            { "legacy_mask", cpmLegacyMask }, { "legacy_mask_v2", cpmLegacyMaskV2 }, { "legacy_mask_v3", cpmLegacyMaskV3 },
            { "loop_strict", cpmLoopStrict },
            { "loop_tolerant", cpmLoopTolerant }, { "arc_coverage", cpmArcCoverage } };
        for (const auto &kv : values) {
            DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
            cfg.set_deserialize_strict("combine_perimeters_method", kv.first);
            CHECK(cfg.opt_enum<CombinePerimetersMethod>("combine_perimeters_method") == kv.second);
            CHECK(cfg.opt_serialize("combine_perimeters_method") == kv.first);
        }
    }
    GIVEN("the max shift value") {
        DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
        CHECK(cfg.option<ConfigOptionFloatOrPercent>("combine_perimeters_max_shift")->percent);
        CHECK(cfg.option<ConfigOptionFloatOrPercent>("combine_perimeters_max_shift")->value == Approx(20.));
        cfg.set_deserialize_strict("combine_perimeters_max_shift", "25%");
        CHECK(cfg.option<ConfigOptionFloatOrPercent>("combine_perimeters_max_shift")->percent);
        CHECK(cfg.option<ConfigOptionFloatOrPercent>("combine_perimeters_max_shift")->value == Approx(25.));
        cfg.set_deserialize_strict("combine_perimeters_max_shift", "0.1");
        CHECK(! cfg.option<ConfigOptionFloatOrPercent>("combine_perimeters_max_shift")->percent);
        CHECK(cfg.option<ConfigOptionFloatOrPercent>("combine_perimeters_max_shift")->value == Approx(0.1));
    }
    GIVEN("the atomic fragments option") {
        CHECK(! FullPrintConfig().combine_perimeters_atomic_fragments.value);
        DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
        CHECK(! cfg.opt_bool("combine_perimeters_atomic_fragments"));
        cfg.set_deserialize_strict("combine_perimeters_atomic_fragments", "1");
        CHECK(cfg.opt_bool("combine_perimeters_atomic_fragments"));
        CHECK(cfg.opt_serialize("combine_perimeters_atomic_fragments") == "1");
        cfg.set_deserialize_strict("combine_perimeters_atomic_fragments", "0");
        CHECK(! cfg.opt_bool("combine_perimeters_atomic_fragments"));
        // Profile paths: ini round trip and the preset option list.
        const DynamicPrintConfig on = load("combine_perimeters_atomic_fragments = 1\n");
        CHECK(on.opt_bool("combine_perimeters_atomic_fragments"));
        const DynamicPrintConfig off = load("combine_perimeters_atomic_fragments = 0\n");
        CHECK(! off.opt_bool("combine_perimeters_atomic_fragments"));
        const auto &keys = Preset::print_options();
        CHECK(std::find(keys.begin(), keys.end(), "combine_perimeters_atomic_fragments") != keys.end());
    }
}

SCENARIO("Fork: combine_perimeters automatic internal perimeters combination", "[Perimeters]")
{
    const TriangleMesh cube = box_mesh(20, 20, 6, 0, 0, 0);
    const Overrides autos = { { "automatic_internal_perimeters_combination", "1" },
                              { "automatic_internal_perimeters_combination_max_layer_height", "100%" } };
    GIVEN("auto on, N = 4 for both roles, 0.1 mm layers") {
        const std::string gcode = Test::slice({ cube }, cp_config("loop_tolerant", 0.1, 4, "20%", "classic", autos));
        THEN("walls combine in groups of 4 (H = 0.4 = nozzle) for both roles") {
            CHECK(gcode_has_internal_height(gcode, 400, "First internal perimeter"));
            CHECK(gcode_has_internal_height(gcode, 400, "Second internal perimeter"));
            CHECK(! gcode_has_internal_thicker_than(gcode, 400, "First internal perimeter"));
            CHECK(! gcode_has_internal_thicker_than(gcode, 400, "Second internal perimeter"));
        }
    }
    GIVEN("auto on, second internal every-N = 1 (concept 1)") {
        DynamicPrintConfig cfg = cp_config("loop_tolerant", 0.1, 4, "20%", "classic", autos);
        cfg.set_deserialize_strict("second_internal_perimeter_every_layers", "1");
        const std::string gcode = Test::slice({ cube }, cfg);
        THEN("only the first internal role combines") {
            CHECK(gcode_has_internal_thicker_than(gcode, 150, "First internal perimeter"));
            CHECK(! gcode_has_internal_thicker_than(gcode, 150, "Second internal perimeter"));
        }
    }
    GIVEN("a deterministic driver") {
        const TriangleMesh taper = make_frustum(8.f, 5.f, 8.f);
        auto signature = [&](const Print &print) {
            std::vector<std::tuple<size_t, int, int, int64_t>> sig;
            for (size_t i = 0; i < layer_count(print); ++ i)
                for (const EntityInfo &e : layer_entities(print, i))
                    for (const ExtrusionPath *p : e.paths)
                        sig.emplace_back(i, (p->role().is_second_internal_perimeter() ? 2 : p->role().is_first_internal_perimeter() ? 1 : 0), int(std::lround(p->height() * 1000.)), int64_t(p->length()));
            return sig;
        };
        const auto a = cp_slice(taper, cp_config("loop_tolerant", 0.1, 4, "20%", "classic", autos));
        const auto b = cp_slice(taper, cp_config("loop_tolerant", 0.1, 4, "20%", "classic", autos));
        CHECK(signature(a->print) == signature(b->print));
        CHECK(count_thick_paths(a->print) > 0);
    }
}

SCENARIO("Fork: combine_perimeters performance sanity", "[Perimeters]")
{
    // A 50 mm tall cube at 0.1 mm layers with combining enabled finishes within a generous guard.
    const auto t0 = std::chrono::steady_clock::now();
    const auto s  = cp_slice(box_mesh(20, 20, 50, 0, 0, 0), cp_config("loop_tolerant", 0.1, 4, "20%", "classic",
        { { "fill_density", "0%" }, { "top_solid_layers", "1" }, { "bottom_solid_layers", "1" } }));
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    CHECK(layer_count(s->print) == 500);
    CHECK(count_thick_paths(s->print) > 0);
    CHECK(secs < 60.);
}

SCENARIO("Fork: combine_perimeters frustum wall angles at 0.1 mm layers", "[Perimeters]")
{
    // Wall angle from vertical: the horizontal shift per 0.1 mm layer is 0.1 * tan(angle); with a 20%
    // shift (0.09 mm) N = 2 groups combine up to about 42 degrees and not at 45 degrees.
    for (const char *method : kLoopMethods) {
        CAPTURE(method);
        for (double angle : { 10., 20., 30., 45. }) {
            CAPTURE(angle);
            const double height = 4.;
            const float  top    = float(14. - height * std::tan(angle * PI / 180.));
            const std::string gcode = Test::slice({ make_frustum(14.f, top, float(height)) }, cp_config(method, 0.1, 2));
            if (angle < 40.)
                CHECK(gcode_has_internal_height(gcode, 200));
            else
                CHECK(! gcode_has_internal_height(gcode, 200));
        }
    }
}

namespace {

// Per (layer, island, perimeter_index) group of >= 2 open internal fragments of the top-level islands:
// the number of fragments with a thickened path and without. Mixed = both.
struct FragmentGroups {
    size_t groups = 0, mixed = 0, thick_groups = 0;
};

FragmentGroups fragment_groups(const Print &print)
{
    FragmentGroups out;
    const auto &layers = print.objects().front()->layers();
    for (size_t li = 0; li < layers.size(); ++ li) {
        const double lh = layers[li]->height;
        for (const LayerRegion *lr : layers[li]->regions()) {
            const auto &islands = lr->perimeters().entities;
            for (size_t ii = 0; ii < islands.size(); ++ ii) {
                if (! islands[ii]->is_collection())
                    continue;
                std::map<int, std::pair<size_t, size_t>> by_depth; // perimeter_index -> (thick, thin)
                for (const ExtrusionEntity *c : static_cast<const ExtrusionEntityCollection*>(islands[ii])->entities) {
                    const auto *mp = dynamic_cast<const ExtrusionMultiPath*>(c);
                    if (mp == nullptr)
                        continue;
                    bool internal = ! mp->paths.empty(), thick = false;
                    for (const ExtrusionPath &p : mp->paths) {
                        internal &= is_internal_role(p);
                        thick    |= is_thick(p, lh);
                    }
                    if (! internal || ! mp->paths.front().attributes().perimeter_index.has_value())
                        continue;
                    auto &cnt = by_depth[int(*mp->paths.front().attributes().perimeter_index)];
                    ++ (thick ? cnt.first : cnt.second);
                }
                for (const auto &kv : by_depth)
                    if (kv.second.first + kv.second.second >= 2) {
                        ++ out.groups;
                        out.thick_groups += kv.second.first > 0;
                        out.mixed        += kv.second.first > 0 && kv.second.second > 0;
                    }
            }
        }
    }
    return out;
}

} // namespace

// Loft of a convex CCW polygon at z = 0 shrunk towards its centroid by `top_scale` at z = height (fan-triangulated
// caps). An elongated polygon that narrows to a thin end makes Arachne split its inner walls into open fragments.
static Slic3r::TriangleMesh make_polygon_loft(const std::vector<Vec2f> &poly, float top_scale, float height)
{
    const int n = int(poly.size());
    Vec2f c(0.f, 0.f);
    for (const Vec2f &p : poly)
        c += p / float(n);
    std::vector<Vec3f> v;
    for (const Vec2f &p : poly)
        v.emplace_back(p.x(), p.y(), 0.f);
    for (const Vec2f &p : poly) {
        const Vec2f q = c + (p - c) * top_scale;
        v.emplace_back(q.x(), q.y(), height);
    }
    std::vector<Vec3i> f;
    for (int i = 1; i + 1 < n; ++ i) {
        f.emplace_back(0, i + 1, i);
        f.emplace_back(n, n + i, n + i + 1);
    }
    for (int i = 0; i < n; ++ i) {
        const int a = i, b = (i + 1) % n;
        f.emplace_back(a, b, b + n);
        f.emplace_back(a, b + n, a + n);
    }
    return Slic3r::TriangleMesh(std::move(v), std::move(f));
}

static TriangleMesh make_plus(float len, float w, float top_scale, float h)
{
    TriangleMesh a = make_polygon_loft({ Vec2f(-len, -w), Vec2f(len, -w), Vec2f(len, w), Vec2f(-len, w) }, top_scale, h);
    TriangleMesh b = make_polygon_loft({ Vec2f(-w, -len), Vec2f(w, -len), Vec2f(w, len), Vec2f(-w, len) }, top_scale, h);
    a.merge(b);
    return a;
}

SCENARIO("Fork: combine_perimeters atomic fragments", "[Perimeters]")
{
    // A plus sign of thin arms: Arachne splits its inner walls into several open fragments per island and
    // depth. With the option off a wall line can be partly combined; with it on, the fragments of one
    // (island, depth) group are combined all together or not at all.
    struct Shape { float w, top_scale; };
    size_t mixed_off_strict_tolerant = 0;
    for (const Shape &shape : { Shape{ 1.1f, 1.f }, Shape{ 1.1f, 0.85f }, Shape{ 0.8f, 0.85f } }) {
        const TriangleMesh plus = make_plus(15.f, shape.w, shape.top_scale, 8.f);
        for (const char *method : { "loop_strict", "loop_tolerant", "legacy_mask_v3" }) {
            CAPTURE(shape.w, shape.top_scale, method);
            const std::string shift = std::string(method) == "legacy_mask_v3" ? "50%" : "20%";
            const auto off = cp_slice(plus, cp_config(method, 0.1, 2, shift, "arachne"));
            const auto on  = cp_slice(plus, cp_config(method, 0.1, 2, shift, "arachne", { { "combine_perimeters_atomic_fragments", "1" } }));
            const FragmentGroups go = fragment_groups(off->print), gn = fragment_groups(on->print);
            INFO("off: groups " << go.groups << " thick " << go.thick_groups << " mixed " << go.mixed
                 << "; on: groups " << gn.groups << " thick " << gn.thick_groups << " mixed " << gn.mixed);
            CHECK(go.groups > 0);
            CHECK(gn.groups == go.groups);
            CHECK(gn.mixed == 0);
            CHECK(count_thick_paths(on->print) <= count_thick_paths(off->print));
            if (std::string(method) != "legacy_mask_v3")
                mixed_off_strict_tolerant += go.mixed;
        }
    }
    // Non-vacuous: without the option the whole-loop methods leave partly combined wall lines.
    CHECK(mixed_off_strict_tolerant > 0);
}

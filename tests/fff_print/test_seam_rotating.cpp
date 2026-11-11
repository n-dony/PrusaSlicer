// Rotating seam (seam_position = rotating): pure functions, G-code integration and config round trip.

#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp>

#include <algorithm>
#include <limits>
#include <boost/filesystem.hpp>
#include <cmath>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "libslic3r/Config.hpp"
#include "libslic3r/GCode/SeamPlacer.hpp"
#include "libslic3r/GCode/SeamRotating.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include "test_data.hpp"

using namespace Slic3r;
using namespace Slic3r::Seams::Rotating;
using Catch::Approx;

namespace {

std::size_t min_cyclic_gap(const std::vector<std::size_t> &order)
{
    const std::size_t n = order.size();
    if (n < 2)
        return n;
    std::size_t best = n;
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t a = order[i], b = order[(i + 1) % n];
        const std::size_t d = a > b ? a - b : b - a;
        best = std::min(best, std::min(d, n - d));
    }
    return best;
}

// Exhaustive maximum of the minimum cyclic gap over permutations starting with 0.
void brute(std::vector<std::size_t> &seq, std::vector<bool> &used, std::size_t n, std::size_t &best)
{
    if (seq.size() == n) {
        best = std::max(best, min_cyclic_gap(seq));
        return;
    }
    for (std::size_t c = 1; c < n; ++c)
        if (!used[c]) {
            used[c] = true;
            seq.push_back(c);
            brute(seq, used, n, best);
            seq.pop_back();
            used[c] = false;
        }
}

double wrap180(double a)
{
    a = std::fmod(a, 360.0);
    if (a > 180.0)
        a -= 360.0;
    if (a <= -180.0)
        a += 360.0;
    return a;
}

struct Level
{
    std::vector<Vec2d> starts; // start XY of every external perimeter run
    std::vector<Vec2d> points; // all external perimeter end points
};

// One Level per ;LAYER_CHANGE. A run start is the nozzle position before an external perimeter extrusion that
// does not directly continue an external perimeter extrusion.
std::vector<Level> parse_levels(const std::string &gcode)
{
    std::vector<Level> levels;
    std::istringstream in(gcode);
    std::string line, role;
    Vec2d pos(0., 0.);
    bool prev_ext = false;
    auto coord = [](const std::string &l, char axis, double &out) {
        const size_t p = l.find(std::string(" ") + axis);
        if (p == std::string::npos)
            return false;
        out = std::stod(l.substr(p + 2));
        return true;
    };
    while (std::getline(in, line)) {
        if (line.rfind(";LAYER_CHANGE", 0) == 0) {
            levels.emplace_back();
            prev_ext = false;
            continue;
        }
        if (line.rfind(";TYPE:", 0) == 0) {
            role = line.substr(6);
            continue;
        }
        if (line.rfind("G1 ", 0) == 0 || line.rfind("G0 ", 0) == 0) {
            const bool extrudes = line.rfind("G1 ", 0) == 0 && line.find(" E") != std::string::npos &&
                                  line.find(" E-") == std::string::npos;
            const bool ext = extrudes && role == "External perimeter" && !levels.empty();
            double x = pos.x(), y = pos.y();
            coord(line, 'X', x);
            coord(line, 'Y', y);
            if (ext) {
                if (!prev_ext)
                    levels.back().starts.push_back(pos);
                levels.back().points.emplace_back(x, y);
            }
            prev_ext = ext;
            pos = Vec2d(x, y);
        }
    }
    return levels;
}

Vec2d bbox_center(const std::vector<Vec2d> &pts)
{
    Vec2d mn = pts.front(), mx = pts.front();
    for (const Vec2d &p : pts) {
        mn = mn.cwiseMin(p);
        mx = mx.cwiseMax(p);
    }
    return (mn + mx) / 2.0;
}

double angle_deg(const Vec2d &p, const Vec2d &c)
{
    return std::atan2(p.y() - c.y(), p.x() - c.x()) * 180.0 / M_PI;
}

// Angle (degrees, relative to the layer centre) of the outer seam of every level.
std::vector<double> outer_angles(const std::vector<Level> &levels, const Vec2d *fixed_center = nullptr)
{
    std::vector<double> result;
    for (const Level &lv : levels) {
        if (lv.starts.empty() || lv.points.empty())
            continue;
        const Vec2d c = fixed_center ? *fixed_center : bbox_center(lv.points);
        // The outer perimeter is the run start farthest from the centre.
        Vec2d best = lv.starts.front();
        for (const Vec2d &s : lv.starts)
            if ((s - c).norm() > (best - c).norm())
                best = s;
        result.push_back(angle_deg(best, c));
    }
    return result;
}

// Seam angle of the outer loop of every level: the start of the first external perimeter extrusion after an
// explicit ";TYPE:External perimeter" tag (a carried-over role at the beginning of a level is ignored).
std::vector<double> tagged_seam_angles(const std::string &gcode)
{
    std::vector<Vec2d> seams;
    std::vector<std::vector<Vec2d>> points;
    std::istringstream in(gcode);
    std::string line;
    Vec2d pos(0., 0.);
    bool in_ext = false, want_start = false;
    while (std::getline(in, line)) {
        if (line.rfind(";LAYER_CHANGE", 0) == 0) {
            seams.emplace_back(0., 0.);
            points.emplace_back();
            want_start = in_ext;  // the role tag is only repeated when it changes
            continue;
        }
        if (line.rfind(";TYPE:", 0) == 0) {
            in_ext = line.substr(6) == "External perimeter";
            want_start = in_ext;
            continue;
        }
        if (line.rfind("G1 ", 0) == 0 || line.rfind("G0 ", 0) == 0) {
            double x = pos.x(), y = pos.y();
            auto coord = [&](char axis, double &out) {
                const size_t q = line.find(std::string(" ") + axis);
                if (q != std::string::npos)
                    out = std::stod(line.substr(q + 2));
            };
            coord('X', x);
            coord('Y', y);
            // Unretract moves (E only) are not extrusions.
            const bool extrudes = line.rfind("G1 ", 0) == 0 && line.find(" E") != std::string::npos &&
                                  line.find(" E-") == std::string::npos &&
                                  (line.find(" X") != std::string::npos || line.find(" Y") != std::string::npos);
            if (extrudes && in_ext && !points.empty()) {
                if (want_start) {
                    seams.back() = pos;
                    want_start = false;
                }
                points.back().emplace_back(x, y);
            }
            pos = Vec2d(x, y);
        }
    }
    std::vector<double> result;
    for (std::size_t i = 0; i < seams.size(); ++i)
        if (!points[i].empty())
            result.push_back(angle_deg(seams[i], bbox_center(points[i])));
    return result;
}

DynamicPrintConfig rotating_config(std::initializer_list<ConfigBase::SetDeserializeItem> extra)
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config_with({
        {"skirts", 0},
        {"perimeters", 2},
        {"seam_position", "rotating"},
        {"scarf_seam_length", 0},
        {"cooling", "0"},
        {"seam_rotation_define_by", "count"},
        {"seam_rotation_count", 4},
        {"seam_rotation_order", "spread"},
    });
    cfg.set_deserialize_strict(extra);
    return cfg;
}

constexpr double angle_tol = 3.0; // max_detour is min(1 mm, 5% of length): ~1.9 deg at r = 30; measured <0.4 deg.

} // namespace

TEST_CASE("Rotating seam spread order table", "[SeamRotating]")
{
    CHECK(spread_order(1) == std::vector<std::size_t>{0});
    CHECK(spread_order(2) == std::vector<std::size_t>{0, 1});
    CHECK(spread_order(4) == std::vector<std::size_t>{0, 2, 1, 3});
    CHECK(spread_order(5) == std::vector<std::size_t>{0, 2, 4, 1, 3});
    CHECK(spread_order(6) == std::vector<std::size_t>{0, 2, 5, 3, 1, 4});
    CHECK(spread_order(7) == std::vector<std::size_t>{0, 3, 6, 2, 5, 1, 4});
    CHECK(spread_order(8) == std::vector<std::size_t>{0, 3, 6, 2, 7, 4, 1, 5});
}

TEST_CASE("Rotating seam spread order properties for N = 1..36", "[SeamRotating]")
{
    for (std::size_t n = 1; n <= 36; ++n) {
        const std::vector<std::size_t> order = spread_order(n);
        INFO("N = " << n);
        REQUIRE(order.size() == n);
        CHECK(order.front() == 0);
        std::vector<std::size_t> sorted = order;
        std::sort(sorted.begin(), sorted.end());
        std::vector<std::size_t> iota(n);
        std::iota(iota.begin(), iota.end(), 0);
        CHECK(sorted == iota);
        CHECK(min_cyclic_gap(order) >= min_cyclic_gap(sequential_order(n)));
        if (n <= 9) {
            std::size_t best = 0;
            std::vector<std::size_t> seq{0};
            std::vector<bool> used(n, false);
            used[0] = true;
            brute(seq, used, n, best);
            CHECK(min_cyclic_gap(order) == best);
        }
    }
}

TEST_CASE("Rotating seam sequential order", "[SeamRotating]")
{
    CHECK(sequential_order(0).empty());
    CHECK(sequential_order(4) == std::vector<std::size_t>{0, 1, 2, 3});
}

TEST_CASE("Rotating seam angle formula", "[SeamRotating]")
{
    Params p;
    p.define_by = srdCount;
    p.count = 4;
    p.order = sroSpread;
    const double expected_spread[] = {0, 180, 90, 270};
    for (std::size_t k = 0; k < 12; ++k)
        CHECK(rotation_angle_deg(k, p) == Approx(expected_spread[k % 4]).margin(1e-9));

    p.order = sroSequential;
    for (std::size_t k = 0; k < 12; ++k)
        CHECK(rotation_angle_deg(k, p) == Approx(90.0 * (k % 4)).margin(1e-9));

    p.order = sroSpread;
    p.start_deg = 45.0;
    p.drift_deg = 3.0;
    for (std::size_t k = 0; k < 12; ++k)
        CHECK(rotation_angle_deg(k, p) == Approx(45.0 + expected_spread[k % 4] + 3.0 * k).margin(1e-9));

    // Degrees mode: always sequential, ignores count and order.
    Params d;
    d.define_by = srdDegrees;
    d.step_deg = 7.0;
    d.order = sroSpread;
    d.count = 5;
    for (std::size_t k = 0; k < 10; ++k)
        CHECK(rotation_angle_deg(k, d) == Approx(7.0 * k).margin(1e-9));
    d.start_deg = 10.0;
    d.drift_deg = 1.5;
    for (std::size_t k = 0; k < 10; ++k)
        CHECK(rotation_angle_deg(k, d) == Approx(10.0 + 8.5 * k).margin(1e-9));
}

TEST_CASE("Rotating seam effective positions", "[SeamRotating]")
{
    Params p;
    p.define_by = srdCount;
    p.count = 6;
    CHECK(effective_positions(p) == 6);

    p.define_by = srdDegrees;
    p.drift_deg = 0;
    const std::pair<double, std::size_t> table[] = {
        {90, 4}, {120, 3}, {72, 5}, {45, 8}, {180, 2}, {7, 360}, {60, 6}, {0.5, 360}, {135, 8}};
    for (const auto &[step, n] : table) {
        p.step_deg = step;
        INFO("step " << step);
        CHECK(effective_positions(p) == n);
    }
    p.step_deg = 117;
    p.drift_deg = 3;
    CHECK(effective_positions(p) == 3);
}

TEST_CASE("Rotating seam object frame yaw", "[SeamRotating]")
{
    {
        const ObjectFrame f = object_frame_yaw(Transform3d::Identity());
        CHECK(f.yaw == Approx(0.0).margin(1e-12));
        CHECK_FALSE(f.mirrored);
    }
    {
        Transform3d t = Transform3d::Identity();
        t.rotate(Eigen::AngleAxisd(M_PI / 2.0, Vec3d::UnitZ()));
        t.translate(Vec3d(10, 20, 5));
        const ObjectFrame f = object_frame_yaw(t);
        CHECK(f.yaw == Approx(M_PI / 2.0).margin(1e-9));
        CHECK_FALSE(f.mirrored);
    }
    {
        Transform3d t = Transform3d::Identity();
        t.scale(Vec3d(-1, 1, 1));
        const ObjectFrame f = object_frame_yaw(t);
        CHECK(f.mirrored);
    }
    // Property: the bed direction of an object-frame angle equals the direction of the transformed vector,
    // for rotations, uniform scales and mirrors (about X or Y) combined.
    for (const double rot : {0.0, 0.4, 1.9, -2.5}) {
        for (const int mirror : {0, 1, 2}) {
            for (const double s : {1.0, 2.5}) {
                Transform3d t = Transform3d::Identity();
                t.translate(Vec3d(30, -4, 0));
                t.rotate(Eigen::AngleAxisd(rot, Vec3d::UnitZ()));
                t.scale(Vec3d(mirror == 1 ? -s : s, mirror == 2 ? -s : s, s));
                const ObjectFrame f = object_frame_yaw(t);
                INFO("rot " << rot << " mirror " << mirror << " scale " << s);
                CHECK(f.mirrored == (mirror != 0));
                for (const double a : {0.0, 30.0, 90.0, 200.0, 311.0}) {
                    const double r = a * M_PI / 180.0;
                    const Vec3d v = t.linear() * Vec3d(std::cos(r), std::sin(r), 0.0);
                    const double bed = to_bed_angle_rad(a, f);
                    const Vec2d want = Vec2d(v.x(), v.y()).normalized();
                    CHECK(std::cos(bed) == Approx(want.x()).margin(1e-9));
                    CHECK(std::sin(bed) == Approx(want.y()).margin(1e-9));
                }
            }
        }
    }
}

TEST_CASE("Rotating seam integration N=4 spread and sequential", "[SeamRotating][Integration]")
{
    const double spread[] = {0, 180, 90, 270};
    const std::vector<Level> a = parse_levels(
        Test::slice({make_cylinder(30., 6.)}, rotating_config({{"seam_rotation_order", "spread"}})));
    const std::vector<double> ang = outer_angles(a);
    REQUIRE(ang.size() >= 8);
    for (std::size_t k = 0; k < 8; ++k) {
        INFO("spread layer " << k);
        CHECK(std::abs(wrap180(ang[k] - spread[k % 4])) < angle_tol);
    }

    const std::vector<Level> b = parse_levels(
        Test::slice({make_cylinder(30., 6.)}, rotating_config({{"seam_rotation_order", "sequential"}})));
    const std::vector<double> seq = outer_angles(b);
    REQUIRE(seq.size() >= 8);
    for (std::size_t k = 0; k < 8; ++k) {
        INFO("sequential layer " << k);
        CHECK(std::abs(wrap180(seq[k] - 90.0 * (k % 4))) < angle_tol);
    }
}

TEST_CASE("Rotating seam integration drift and start offset", "[SeamRotating][Integration]")
{
    const double spread[] = {0, 180, 90, 270};
    const std::vector<double> drift = outer_angles(parse_levels(Test::slice(
        {make_cylinder(30., 6.)}, rotating_config({{"seam_rotation_drift", 3}}))));
    REQUIRE(drift.size() >= 12);
    for (std::size_t k = 0; k < 12; ++k) {
        INFO("drift layer " << k);
        CHECK(std::abs(wrap180(drift[k] - (spread[k % 4] + 3.0 * k))) < angle_tol);
    }

    const std::vector<double> start = outer_angles(parse_levels(Test::slice(
        {make_cylinder(30., 6.)}, rotating_config({{"seam_rotation_start", 45}}))));
    REQUIRE(start.size() >= 8);
    for (std::size_t k = 0; k < 8; ++k) {
        INFO("start layer " << k);
        CHECK(std::abs(wrap180(start[k] - (45.0 + spread[k % 4]))) < angle_tol);
    }
}

TEST_CASE("Rotating seam integration degrees mode", "[SeamRotating][Integration]")
{
    const std::vector<double> ang = outer_angles(parse_levels(Test::slice(
        {make_cylinder(30., 6.)},
        rotating_config({{"seam_rotation_define_by", "degrees"}, {"seam_rotation_step", 7}}))));
    REQUIRE(ang.size() >= 12);
    for (std::size_t k = 0; k < 12; ++k) {
        INFO("degrees layer " << k);
        CHECK(std::abs(wrap180(ang[k] - 7.0 * k)) < angle_tol);
    }
}

TEST_CASE("Rotating seam integration rotated instance", "[SeamRotating][Integration]")
{
    // One object, two instances; the second is rotated 90 degrees about Z, so it becomes its own PrintObject.
    DynamicPrintConfig cfg = rotating_config({});
    Model model;
    ModelObject *object = model.add_object();
    object->name = "cylinder.stl";
    object->add_volume(make_cylinder(30., 6.));
    ModelInstance *i1 = object->add_instance();
    i1->set_offset(Vec3d(60, 100, 0));
    ModelInstance *i2 = object->add_instance();
    i2->set_offset(Vec3d(140, 100, 0));
    i2->set_rotation(Vec3d(0, 0, M_PI / 2.0));
    object->ensure_on_bed();

    Print print;
    print.auto_assign_extruders(object);
    print.apply(model, cfg);
    print.validate();
    print.set_status_silent();
    const std::vector<Level> levels = parse_levels(Test::gcode(print));
    REQUIRE(levels.size() >= 6);

    const Vec2d c1(60, 100), c2(140, 100);
    std::size_t compared = 0;
    for (const Level &lv : levels) {
        Vec2d s1, s2;
        bool f1 = false, f2 = false;
        for (const Vec2d &s : lv.starts) {
            // Outer runs only: radius ~30 around the instance centre.
            // The first run start in the band is the loop seam; later starts are mid-loop path breaks.
            if (!f1 && std::abs((s - c1).norm() - 30.0) < 3.0) { s1 = s; f1 = true; }
            if (!f2 && std::abs((s - c2).norm() - 30.0) < 3.0) { s2 = s; f2 = true; }
        }
        REQUIRE(f1);
        REQUIRE(f2);
        const double diff = wrap180(angle_deg(s2, c2) - angle_deg(s1, c1));
        INFO("level " << compared);
        CHECK(std::abs(diff - 90.0) < angle_tol);
        ++compared;
    }
    CHECK(compared >= 6);
}

TEST_CASE("Rotating seam integration hole offset", "[SeamRotating][Integration]")
{
    for (const char *offset : {"opposite", "same"}) {
        DynamicPrintConfig cfg = rotating_config({{"seam_rotation_hole_offset", offset}});
        Model model;
        ModelObject *object = model.add_object();
        object->name = "tube.stl";
        object->add_volume(make_cylinder(30., 6.));
        TriangleMesh hole = make_cylinder(15., 8.);
        hole.translate(0.f, 0.f, -1.f);
        ModelVolume *neg = object->add_volume(std::move(hole), ModelVolumeType::NEGATIVE_VOLUME);
        (void) neg;
        object->add_instance()->set_offset(Vec3d(100, 100, 0));
        object->ensure_on_bed();

        Print print;
        print.auto_assign_extruders(object);
        print.apply(model, cfg);
        print.validate();
        print.set_status_silent();
        const std::vector<Level> levels = parse_levels(Test::gcode(print));
        REQUIRE(levels.size() >= 8);

        const Vec2d c(100, 100);
        std::size_t checked = 0;
        for (std::size_t k = 0; k < 8; ++k) {
            const Level &lv = levels[k];
            bool fo = false, fh = false;
            double ao = 0, ah = 0;
            for (const Vec2d &s : lv.starts) {
                const double r = (s - c).norm();
                if (!fo && std::abs(r - 30.0) < 3.0) { ao = angle_deg(s, c); fo = true; }
                if (!fh && std::abs(r - 15.0) < 3.0) { ah = angle_deg(s, c); fh = true; }
            }
            INFO("offset " << offset << " layer " << k);
            REQUIRE(fo);
            REQUIRE(fh);
            const double want = std::string(offset) == "opposite" ? 180.0 : 0.0;
            // Hole radius 15: 1 mm max detour is ~3.8 deg, so allow more than for the outer wall.
            CHECK(std::abs(wrap180(ah - ao - want)) < 6.0);
            ++checked;
        }
        CHECK(checked == 8);
    }
}

TEST_CASE("Rotating seam group sub-layer counting", "[SeamRotating][Integration]")
{
    // external_perimeter_sublayers = 3 prints two pure perimeter sub-layers under every top level; in group
    // mode all three levels of a group share one seam index.
    DynamicPrintConfig cfg = rotating_config({
        {"layer_height", 0.3},
        {"first_layer_height", 0.3},
        {"perimeter_generator", "arachne"},
        {"external_perimeter_sublayers", 3},
        {"seam_rotation_sublayers", "group"},
    });
    const std::vector<double> group = outer_angles(parse_levels(Test::slice({make_cylinder(30., 4.)}, cfg)));
    const double spread[] = {0, 180, 90, 270};
    REQUIRE(group.size() > 10);
    REQUIRE((group.size() - 1) % 3 == 0);
    for (std::size_t j = 0; j < group.size(); ++j) {
        const std::size_t k = j == 0 ? 0 : (j - 1) / 3 + 1;
        INFO("group level " << j);
        CHECK(std::abs(wrap180(group[j] - spread[k % 4])) < angle_tol);
    }
}

TEST_CASE("Rotating seam loop mode without sub-layers advances every layer", "[SeamRotating][Integration]")
{
    const std::vector<double> loop = outer_angles(parse_levels(Test::slice(
        {make_cylinder(30., 6.)}, rotating_config({{"seam_rotation_sublayers", "loop"}}))));
    const double spread[] = {0, 180, 90, 270};
    REQUIRE(loop.size() >= 8);
    for (std::size_t k = 0; k < 8; ++k) {
        INFO("loop layer " << k);
        CHECK(std::abs(wrap180(loop[k] - spread[k % 4])) < angle_tol);
    }
}

TEST_CASE("Rotating seam config round trip", "[SeamRotating][Config]")
{
    const std::vector<std::pair<std::string, std::string>> values = {
        {"seam_position", "rotating"},
        {"seam_rotation_define_by", "degrees"},
        {"seam_rotation_count", "7"},
        {"seam_rotation_step", "12.5"},
        {"seam_rotation_start", "33"},
        {"seam_rotation_order", "sequential"},
        {"seam_rotation_drift", "-2.5"},
        {"seam_rotation_hole_offset", "same"},
        {"seam_rotation_sublayers", "loop"},
    };
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
    for (const auto &[key, value] : values)
        cfg.set_deserialize_strict(key, value);

    DynamicPrintConfig copy = DynamicPrintConfig::full_print_config();
    for (const auto &[key, value] : values) {
        INFO(key);
        CHECK(cfg.opt_serialize(key) == value);
        copy.set_deserialize_strict(key, cfg.opt_serialize(key));
        CHECK(copy.opt_serialize(key) == value);
    }
    CHECK(copy.opt_enum<SeamPosition>("seam_position") == spRotating);
    const Params p = Params::from_config(copy);
    CHECK(p.define_by == srdDegrees);
    CHECK(p.count == 7);
    CHECK(p.step_deg == Approx(12.5));
    CHECK(p.start_deg == Approx(33.));
    CHECK(p.order == sroSequential);
    CHECK(p.drift_deg == Approx(-2.5));
    CHECK_FALSE(p.hole_opposite);
    CHECK(p.sublayers == srsLoop);

    // Defaults.
    const DynamicPrintConfig def = DynamicPrintConfig::full_print_config();
    const Params d = Params::from_config(def);
    CHECK(d.define_by == srdCount);
    CHECK(d.order == sroSpread);
    CHECK(d.hole_opposite);
    CHECK(d.sublayers == srsGroup);
    CHECK(d.drift_deg == Approx(0.));
    CHECK(def.opt_enum<SeamPosition>("seam_position") != spRotating);
}

TEST_CASE("Rotating seam invalid config values are rejected", "[SeamRotating][Config]")
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
    CHECK_THROWS(cfg.set_deserialize_strict("seam_position", "spinning"));
    CHECK_THROWS(cfg.set_deserialize_strict("seam_rotation_define_by", "radians"));
    CHECK_THROWS(cfg.set_deserialize_strict("seam_rotation_order", "random"));
    CHECK_THROWS(cfg.set_deserialize_strict("seam_rotation_hole_offset", "left"));
    CHECK_THROWS(cfg.set_deserialize_strict("seam_rotation_sublayers", "both"));
    CHECK_THROWS(cfg.set_deserialize_strict("seam_rotation_count", "many"));
    CHECK_THROWS(cfg.set_deserialize_strict("seam_rotation_step", "abc"));
    // Range limits are declared on the option definitions (enforced by the UI and by validate()).
    const ConfigOptionDef *count = print_config_def.get("seam_rotation_count");
    REQUIRE(count != nullptr);
    CHECK(count->min == 1);
    CHECK(count->max == 36);
    const ConfigOptionDef *step = print_config_def.get("seam_rotation_step");
    CHECK(step->min == Approx(0.01));
    CHECK(step->max == Approx(359.99));
    const ConfigOptionDef *drift = print_config_def.get("seam_rotation_drift");
    CHECK(drift->min == Approx(-45));
    CHECK(drift->max == Approx(45));
    // from_config clamps the count.
    cfg.set_deserialize_strict("seam_rotation_count", "100");
    CHECK(Params::from_config(cfg).count == 36);
}

TEST_CASE("Rotating seam keys appear in the G-code footer", "[SeamRotating][Config]")
{
    const std::string gcode = Test::slice({make_cylinder(10., 1.)}, rotating_config({
        {"seam_rotation_define_by", "degrees"},
        {"seam_rotation_step", 12.5},
        {"seam_rotation_count", 7},
        {"seam_rotation_start", 33},
        {"seam_rotation_order", "sequential"},
        {"seam_rotation_drift", 2},
        {"seam_rotation_hole_offset", "same"},
        {"seam_rotation_sublayers", "loop"},
    }));
    CHECK(Test::contains(gcode, "; seam_position = rotating"));
    CHECK(Test::contains(gcode, "; seam_rotation_define_by = degrees"));
    CHECK(Test::contains(gcode, "; seam_rotation_count = 7"));
    CHECK(Test::contains(gcode, "; seam_rotation_step = 12.5"));
    CHECK(Test::contains(gcode, "; seam_rotation_start = 33"));
    CHECK(Test::contains(gcode, "; seam_rotation_order = sequential"));
    CHECK(Test::contains(gcode, "; seam_rotation_drift = 2"));
    CHECK(Test::contains(gcode, "; seam_rotation_hole_offset = same"));
    CHECK(Test::contains(gcode, "; seam_rotation_sublayers = loop"));
}

TEST_CASE("rotating loop mode advances the angle per sub-pass", "[SeamRotating][Integration]")
{
    // First layer k = 1 (level 0); then groups of 3 levels (2 lower sub-passes, then the top level).
    // Loop: every level is one virtual pass in print order. Group: the 3 levels of a group share an angle.
    auto run = [](const char *mode) {
        return tagged_seam_angles(Test::slice({make_cylinder(30., 4.)}, rotating_config({
            {"layer_height", 0.3},
            {"first_layer_height", 0.3},
            {"perimeters", 3},
            {"perimeter_generator", "arachne"},
            {"external_perimeter_sublayers", 3},
            {"seam_rotation_order", "sequential"},
            {"seam_rotation_sublayers", mode},
        })));
    };
    const std::vector<double> loop = run("loop");
    const std::vector<double> group = run("group");
    REQUIRE(loop.size() > 10);
    REQUIRE(group.size() == loop.size());
    std::ostringstream seq;
    for (std::size_t j = 0; j < loop.size(); ++j) {
        seq << wrap180(loop[j]) << ' ';
        INFO("loop level " << j);
        CHECK(std::abs(wrap180(loop[j] - 90.0 * double(j % 4))) < angle_tol);
        const std::size_t g = j == 0 ? 0 : (j - 1) / 3 + 1;
        INFO("group level " << j);
        CHECK(std::abs(wrap180(group[j] - 90.0 * double(g % 4))) < angle_tol);
    }
    INFO("loop angles: " << seq.str());
    // First layer is one level; every later layer has 3 levels (2 sub-passes + the top).
    CHECK((loop.size() - 1) % 3 == 0);
    CHECK(loop.size() == group.size());
}

TEST_CASE("Rotating seam minimum seam gap", "[SeamRotating]")
{
    Params p;
    p.define_by = srdCount;
    p.count = 4;
    p.order = sroSpread; // order {0,2,1,3}: min cyclic gap is 1 slot
    CHECK(min_seam_gap_rad(p) == Approx(M_PI / 2).margin(1e-12));
    p.order = sroSequential;
    CHECK(min_seam_gap_rad(p) == Approx(M_PI / 2).margin(1e-12));
    p.count = 1;
    CHECK(min_seam_gap_rad(p) == Approx(2 * M_PI));
    // Count mode: the per-layer drift adds to every consecutive delta.
    p.count = 4;
    p.order = sroSequential;
    p.drift_deg = -45.;
    CHECK(min_seam_gap_rad(p) == Approx(M_PI / 4).margin(1e-12)); // deltas of 90 - 45 degrees
    p.drift_deg = 10.;
    CHECK(min_seam_gap_rad(p) == Approx(100. * M_PI / 180.).margin(1e-12));
    p.drift_deg = 0.;
    p.define_by = srdDegrees;
    p.step_deg = 7.;
    p.drift_deg = 0.;
    CHECK(min_seam_gap_rad(p) == Approx(7. * M_PI / 180.).margin(1e-12));
    p.step_deg = 200.; // folded to 160
    CHECK(min_seam_gap_rad(p) == Approx(160. * M_PI / 180.).margin(1e-12));
    p.step_deg = 7.;
    p.drift_deg = 1.;
    CHECK(min_seam_gap_rad(p) == Approx(8. * M_PI / 180.).margin(1e-12));
}

TEST_CASE("Rotating seam scarf guard predicate", "[SeamRotating]")
{
    // r = 30 mm: circumference ~188.5 mm, gap pi/2 -> half gap arc = 23.6 mm.
    const double circ = 2 * M_PI * 30.;
    CHECK_FALSE(scarf_guard_blocks(5., circ, M_PI / 2));
    CHECK(scarf_guard_blocks(40., circ, M_PI / 2));
    CHECK_FALSE(scarf_guard_blocks(23., circ, M_PI / 2));
    CHECK(scarf_guard_blocks(24., circ, M_PI / 2));
}

namespace {
// Number of extruding G1 moves that also change Z (scarf ramp segments).
std::size_t scarf_ramp_moves(const std::string &gcode)
{
    std::istringstream in(gcode);
    std::string line;
    std::size_t n = 0;
    while (std::getline(in, line))
        if (line.rfind("G1 ", 0) == 0 && line.find(" Z") != std::string::npos && line.find(" E") != std::string::npos)
            ++n;
    return n;
}
} // namespace

TEST_CASE("Rotating seam scarf guard integration", "[SeamRotating][Integration]")
{
    const auto run = [](double len, bool entire) {
        return scarf_ramp_moves(Test::slice(
            {make_cylinder(30., 6.)},
            rotating_config({{"scarf_seam_placement", "everywhere"},
                             {"scarf_seam_only_on_smooth", 0},
                             {"scarf_seam_length", len},
                             {"scarf_seam_entire_loop", entire ? 1 : 0}})));
    };
    CHECK(run(5., false) > 0);
    CHECK(run(40., false) == 0);
    CHECK(run(40., true) > 0);

    // Positive control: without rotating seams a 40 mm scarf keeps its ramp (the zero above is the guard).
    const std::size_t aligned_ramp = scarf_ramp_moves(Test::slice(
        {make_cylinder(30., 6.)},
        rotating_config({{"seam_position", "aligned"},
                         {"scarf_seam_placement", "everywhere"},
                         {"scarf_seam_only_on_smooth", 0},
                         {"scarf_seam_length", 40.},
                         {"scarf_seam_entire_loop", 0}})));
    CHECK(aligned_ramp > 0);
}

TEST_CASE("Rotating seam scarf guard is decided per perimeter group", "[SeamRotating][Integration]")
{
    // N = 4 spread, r = 30 mm: a 23.3 mm scarf is below the external threshold (~23.4 mm) and above the
    // threshold of the shorter inner loops (~23.1 mm / ~22.8 mm). All loops of the group must keep the ramp.
    const auto run = [](int perimeters) {
        return scarf_ramp_moves(Test::slice(
            {make_cylinder(30., 6.)},
            rotating_config({{"perimeters", perimeters},
                             {"scarf_seam_placement", "everywhere"},
                             {"scarf_seam_only_on_smooth", 0},
                             {"scarf_seam_on_inner_perimeters", 1},
                             {"scarf_seam_length", 23.3},
                             {"scarf_seam_entire_loop", 0}})));
    };
    const std::size_t one = run(1);
    const std::size_t three = run(3);
    REQUIRE(one > 0);
    CHECK(three > 2 * one);
}

TEST_CASE("Rotating seam per-object override", "[SeamRotating][Integration]")
{
    Print print;
    Model model;
    Test::init_print({make_cylinder(15., 3.), make_cylinder(15., 3.)}, print, model, rotating_config({}));
    REQUIRE(model.objects.size() == 2);
    // Only the second object gets a start angle of 90 degrees.
    model.objects[1]->config.set_deserialize_strict("seam_rotation_start", "90");
    print.apply(model, rotating_config({}));
    const std::vector<Level> levels = parse_levels(Test::gcode(print));
    REQUIRE(print.objects().size() == 2);
    const double spread[] = {0, 180, 90, 270};
    std::size_t checked = 0;
    for (std::size_t i = 0; i < 2; ++i) {
        const Vec2d c = unscale(print.objects()[i]->instances()[0].shift);
        // The overridden object is the one whose model object carries the key.
        const bool overridden = print.objects()[i]->model_object()->config.has("seam_rotation_start");
        std::size_t k = 0;
        for (const Level &lv : levels) {
            for (const Vec2d &st : lv.starts) {
                if (std::abs((st - c).norm() - 15.) > 3.)
                    continue;
                if (k < 8) {
                    const double want = spread[k % 4] + (overridden ? 90. : 0.);
                    INFO("object " << i << " layer " << k);
                    CHECK(std::abs(wrap180(angle_deg(st, c) - want)) < angle_tol + 1.);
                    ++checked;
                }
                ++k;
                break;
            }
        }
    }
    CHECK(checked == 16);
}

TEST_CASE("Rotating seam group counting with combined perimeters and infill", "[SeamRotating][Integration]")
{
    const double spread[] = {0, 180, 90, 270};
    // Infill-only combining must not hold the seam back: the angle still advances every layer.
    {
        const std::vector<double> ang = outer_angles(parse_levels(Test::slice({make_cylinder(30., 3.)}, rotating_config({
            {"perimeters", 3},
            {"layer_height", 0.15},
            {"first_layer_height", 0.15},
            {"seam_rotation_sublayers", "group"},
            {"infill_every_layers", 2},
            {"first_internal_perimeter_every_layers", 1},
            {"second_internal_perimeter_every_layers", 1},
        }))));
        REQUIRE(ang.size() >= 8);
        for (std::size_t k = 0; k < 8; ++k) {
            INFO("infill-only combine, layer " << k);
            CHECK(std::abs(wrap180(ang[k] - spread[k % 4])) < angle_tol);
        }
    }
    // Perimeter combining (every 2 layers): the angle advances once per group of two layers.
    {
        const std::vector<double> ang = outer_angles(parse_levels(Test::slice({make_cylinder(30., 3.)}, rotating_config({
            {"perimeters", 3},
            {"layer_height", 0.15},
            {"first_layer_height", 0.15},
            {"seam_rotation_sublayers", "group"},
            {"first_internal_perimeter_every_layers", 2},
            {"second_internal_perimeter_every_layers", 2},
        }))));
        REQUIRE(ang.size() >= 9);
        // Layer 0 is a group on its own; afterwards layers (2g-1, 2g) share the angle of group g.
        for (std::size_t k = 1; k + 1 < 9; k += 2) {
            INFO("perimeter combine, layers " << k << "," << k + 1);
            const std::size_t g = (k + 1) / 2;
            CHECK(std::abs(wrap180(ang[k] - spread[g % 4])) < angle_tol);
            CHECK(std::abs(wrap180(ang[k + 1] - spread[g % 4])) < angle_tol);
        }
    }
}

// ---------------------------------------------------------------------------------------------------------
// Half-loop inner stagger (Part B)
// ---------------------------------------------------------------------------------------------------------

namespace {
Linesf square_lines(double a)
{
    const Vec2d p[4] = {{0, 0}, {a, 0}, {a, a}, {0, a}};
    Linesf lines;
    for (int i = 0; i < 4; ++i)
        lines.emplace_back(p[i], p[(i + 1) % 4]);
    return lines;
}

Linesf circle_lines(double r, int n)
{
    Linesf lines;
    for (int i = 0; i < n; ++i) {
        const double a0 = 2 * M_PI * i / n, a1 = 2 * M_PI * (i + 1) / n;
        lines.emplace_back(Vec2d(r * std::cos(a0), r * std::sin(a0)), Vec2d(r * std::cos(a1), r * std::sin(a1)));
    }
    return lines;
}

struct Loop
{
    Vec2d start{0, 0};
    std::vector<Vec2d> points;
};

// Perimeter loops (external and internal) of the layer with the given index (0 based, counted by ;LAYER_CHANGE).
// A loop starts at the first extruding move after a non-extruding move (or a role change).
std::vector<Loop> layer_loops(const std::string &gcode, std::size_t layer)
{
    std::vector<Loop> loops;
    std::istringstream in(gcode);
    std::string line;
    Vec2d pos(0., 0.);
    bool is_perimeter = false, prev_extrudes = false;
    int layer_idx = -1;
    while (std::getline(in, line)) {
        if (line.rfind(";LAYER_CHANGE", 0) == 0) {
            ++layer_idx;
            prev_extrudes = false;
            continue;
        }
        if (line.rfind(";TYPE:", 0) == 0) {
            const std::string role = line.substr(6);
            is_perimeter = role.find("erimeter") != std::string::npos && role.find("Overhang") == std::string::npos;
            prev_extrudes = false;
            continue;
        }
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0)
            continue;
        double x = pos.x(), y = pos.y();
        const auto coord = [&](char axis, double &out) {
            const size_t q = line.find(std::string(" ") + axis);
            if (q != std::string::npos)
                out = std::stod(line.substr(q + 2));
        };
        coord('X', x);
        coord('Y', y);
        const bool extrudes = line.rfind("G1 ", 0) == 0 && line.find(" E") != std::string::npos &&
                              line.find(" E-") == std::string::npos &&
                              (line.find(" X") != std::string::npos || line.find(" Y") != std::string::npos);
        if (extrudes && is_perimeter && layer_idx == int(layer)) {
            if (!prev_extrudes) {
                loops.emplace_back();
                loops.back().start = pos;
            }
            loops.back().points.emplace_back(x, y);
        }
        prev_extrudes = extrudes && is_perimeter;
        pos = Vec2d(x, y);
    }
    // Drop fragments (unretract / wipe leftovers).
    loops.erase(std::remove_if(loops.begin(), loops.end(), [](const Loop &l) { return l.points.size() < 3; }), loops.end());
    return loops;
}

double loop_extent(const Loop &l)
{
    return bbox_center(l.points).x() - std::min_element(l.points.begin(), l.points.end(), [](auto &a, auto &b) { return a.x() < b.x(); })->x();
}

// Loops sorted from the outermost (depth 0) to the innermost.
std::vector<Loop> loops_by_depth(const std::string &gcode, std::size_t layer)
{
    std::vector<Loop> loops = layer_loops(gcode, layer);
    std::sort(loops.begin(), loops.end(), [](const Loop &a, const Loop &b) { return loop_extent(a) > loop_extent(b); });
    return loops;
}

// Position along the rectangle loop perimeter (mm, counter-clockwise from the bottom-left corner of its bbox).
double rect_arc(const Loop &l, const Vec2d &p)
{
    Vec2d mn = l.points.front(), mx = l.points.front();
    for (const Vec2d &q : l.points) {
        mn = mn.cwiseMin(q);
        mx = mx.cwiseMax(q);
    }
    const double w = mx.x() - mn.x(), h = mx.y() - mn.y();
    const double eps = 0.05;
    if (std::abs(p.y() - mn.y()) < eps) return p.x() - mn.x();
    if (std::abs(p.x() - mx.x()) < eps) return w + p.y() - mn.y();
    if (std::abs(p.y() - mx.y()) < eps) return w + h + mx.x() - p.x();
    return 2 * w + h + mx.y() - p.y();
}

DynamicPrintConfig half_loop_config(std::initializer_list<ConfigBase::SetDeserializeItem> extra)
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config_with({
        {"skirts", 0},
        {"perimeters", 4},
        {"seam_position", "aligned"},
        {"cooling", "0"},
        {"scarf_seam_length", 0},
        {"staggered_inner_seams", 1},
        {"staggered_inner_seams_mode", "half_loop"},
    });
    cfg.set_deserialize_strict(extra);
    return cfg;
}

std::string gcode_body(const std::string &gcode)
{
    // Skip the first line: it carries a generation timestamp.
    const std::size_t first = gcode.find('\n');
    const std::size_t pos = gcode.find("; prusaslicer_config = begin");
    return gcode.substr(first == std::string::npos ? 0 : first, pos == std::string::npos ? pos : pos - first);
}
} // namespace

TEST_CASE("Half-loop arc walk helper", "[SeamRotating][HalfLoop]")
{
    using Slic3r::Seams::point_at_arc_length;
    const Linesf sq = square_lines(10.);
    // Half of the square starting at (5, 0) on line 0: arrives at (5, 10) on line 2.
    auto [idx, p] = point_at_arc_length(sq, 0, Vec2d(5, 0), 20.);
    CHECK(idx == 2);
    CHECK((p - Vec2d(5, 10)).norm() < 1e-9);
    // Quarter walk around a corner.
    std::tie(idx, p) = point_at_arc_length(sq, 0, Vec2d(8, 0), 4.);
    CHECK(idx == 1);
    CHECK((p - Vec2d(10, 2)).norm() < 1e-9);
    // Wrap beyond the loop length and negative arcs.
    std::tie(idx, p) = point_at_arc_length(sq, 3, Vec2d(0, 5), 45.);
    CHECK((p - Vec2d(0, 0)).norm() < 1e-9);
    std::tie(idx, p) = point_at_arc_length(sq, 0, Vec2d(2, 0), -4.);
    CHECK((p - Vec2d(0, 2)).norm() < 1e-9);
    CHECK(idx == 3);
    // Zero arc returns the start.
    std::tie(idx, p) = point_at_arc_length(sq, 1, Vec2d(10, 3), 0.);
    CHECK(idx == 1);
    CHECK((p - Vec2d(10, 3)).norm() < 1e-9);

    // Circle: half the circumference is the antipodal point (up to the polygonisation).
    const int n = 720;
    const double r = 20.;
    const Linesf circle = circle_lines(r, n);
    double len = 0;
    for (const Linef &l : circle)
        len += (l.b - l.a).norm();
    std::tie(idx, p) = point_at_arc_length(circle, 0, circle[0].a, len / 2.);
    CHECK((p - Vec2d(-r, 0)).norm() < 1e-6);
    std::tie(idx, p) = point_at_arc_length(circle, 100, circle[100].a, len / 2.);
    CHECK((p + circle[100].a).norm() < 1e-6);
}

namespace {
void check_circle_half_loop(const std::string &gcode, const char *label)
{
    const std::vector<Loop> loops = loops_by_depth(gcode, 3);
    INFO(label);
    REQUIRE(loops.size() == 4);
    const Vec2d c = bbox_center(loops[0].points);
    std::vector<double> ang;
    for (const Loop &l : loops)
        ang.push_back(angle_deg(l.start, c));
    for (int d = 1; d < 4; ++d) {
        const double off = std::abs(wrap180(ang[d] - ang[d - 1])) / 360.0;
        INFO("depth " << d << " offset to previous loop (fraction of loop): " << off);
        if (d % 2 == 1)
            CHECK(std::abs(off - 0.5) < 0.05);
    }
    CHECK(std::abs(wrap180(ang[2] - ang[0])) < 360.0 * 0.05);
    CHECK(std::abs(wrap180(ang[3] - ang[1])) < 360.0 * 0.05);
}
} // namespace

TEST_CASE("Half-loop stagger on a cylinder, Arachne and classic", "[SeamRotating][HalfLoop][Integration]")
{
    check_circle_half_loop(Test::slice({make_cylinder(20., 4.)}, half_loop_config({})), "arachne");
    check_circle_half_loop(
        Test::slice({make_cylinder(20., 4.)}, half_loop_config({{"perimeter_generator", "classic"}})), "classic");
}

TEST_CASE("Half-loop stagger on a rectangular box", "[SeamRotating][HalfLoop][Integration]")
{
    for (const char *gen : {"arachne", "classic"}) {
        INFO(gen);
        const std::vector<Loop> loops = loops_by_depth(
            Test::slice({make_cube(40., 30., 4.)}, half_loop_config({{"perimeter_generator", gen}})), 3);
        REQUIRE(loops.size() == 4);
        std::vector<double> arc, len;
        for (const Loop &l : loops) {
            arc.push_back(rect_arc(l, l.start));
            Vec2d mn = l.points.front(), mx = l.points.front();
            for (const Vec2d &q : l.points) {
                mn = mn.cwiseMin(q);
                mx = mx.cwiseMax(q);
            }
            len.push_back(2 * (mx.x() - mn.x() + mx.y() - mn.y()));
        }
        for (int d = 1; d < 4; d += 2) {
            // Compare along the loop: the odd loop's seam is half its loop away from the previous even loop's.
            double diff = std::fmod(std::abs(arc[d] - arc[d - 1] * len[d] / len[d - 1]), len[d]);
            diff = std::min(diff, len[d] - diff);
            INFO("depth " << d << " arc diff " << diff << " of " << len[d] << " (expected " << len[d] / 2 << ")");
            CHECK(std::abs(diff - len[d] / 2.) < 4.0);
        }
        CHECK((loops[2].start - loops[0].start).norm() < 5.0);
    }
}

TEST_CASE("Half-loop mode is opt-in and depth mode is unchanged", "[SeamRotating][HalfLoop][Integration]")
{
    const auto base = [](std::initializer_list<ConfigBase::SetDeserializeItem> extra) {
        DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config_with(
            {{"skirts", 0}, {"perimeters", 4}, {"seam_position", "aligned"}, {"cooling", "0"}});
        cfg.set_deserialize_strict(extra);
        return gcode_body(Test::slice({make_cylinder(20., 4.)}, cfg));
    };
    const std::string untouched = base({});
    CHECK(base({{"staggered_inner_seams_mode", "depth"}}) == untouched);
    CHECK(base({{"staggered_inner_seams_mode", "half_loop"}}) == untouched); // staggered_inner_seams is off
    const std::string depth_on = base({{"staggered_inner_seams", 1}});
    CHECK(base({{"staggered_inner_seams", 1}, {"staggered_inner_seams_mode", "depth"}}) == depth_on);
    CHECK(base({{"staggered_inner_seams", 1}, {"staggered_inner_seams_mode", "half_loop"}}) != depth_on);
}

// ---------------------------------------------------------------------------------------------------------
// Step 08: inertness, L-shape half-loop, interactions, ini round trip
// ---------------------------------------------------------------------------------------------------------

namespace {
const std::vector<std::string> &seam_rotation_keys()
{
    static const std::vector<std::string> keys = {
        "seam_rotation_define_by", "seam_rotation_count",  "seam_rotation_step",        "seam_rotation_start",
        "seam_rotation_order",     "seam_rotation_drift",  "seam_rotation_hole_offset", "seam_rotation_sublayers"};
    return keys;
}

TriangleMesh make_l_shape(double h)
{
    indexed_triangle_set a = its_make_cube(40., 15., h);
    indexed_triangle_set b = its_make_cube(15., 40., h);
    its_merge(a, std::move(b));
    return TriangleMesh(std::move(a));
}

// Closed polyline of a loop: start followed by all extrusion end points.
std::vector<Vec2d> loop_polyline(const Loop &l)
{
    std::vector<Vec2d> poly;
    poly.push_back(l.start);
    for (const Vec2d &p : l.points)
        poly.push_back(p);
    if ((poly.back() - poly.front()).norm() > 1e-6)
        poly.push_back(poly.front());
    return poly;
}

double polyline_length(const std::vector<Vec2d> &poly)
{
    double len = 0.;
    for (std::size_t i = 1; i < poly.size(); ++i)
        len += (poly[i] - poly[i - 1]).norm();
    return len;
}

// Arc length (from poly.front()) of the point of the polyline nearest to p.
double arc_of_nearest(const std::vector<Vec2d> &poly, const Vec2d &p)
{
    double best_d = std::numeric_limits<double>::max(), best_arc = 0., acc = 0.;
    for (std::size_t i = 1; i < poly.size(); ++i) {
        const Vec2d d = poly[i] - poly[i - 1];
        const double l = d.norm();
        const double t = l > 0. ? std::clamp((p - poly[i - 1]).dot(d) / (l * l), 0., 1.) : 0.;
        const double dist = (poly[i - 1] + t * d - p).norm();
        if (dist < best_d) {
            best_d = dist;
            best_arc = acc + t * l;
        }
        acc += l;
    }
    return best_arc;
}
} // namespace

TEST_CASE("Rotating seam options are inert unless seam_position is rotating", "[SeamRotating][Integration]")
{
    const auto run = [](const char *pos, const char *gen, bool cyl, bool nondefault) {
        DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config_with(
            {{"skirts", 0}, {"perimeters", 2}, {"seam_position", pos}, {"perimeter_generator", gen}, {"cooling", "0"}});
        if (nondefault)
            cfg.set_deserialize_strict({{"seam_rotation_define_by", "degrees"},
                                        {"seam_rotation_count", 7},
                                        {"seam_rotation_step", 12.5},
                                        {"seam_rotation_start", 33},
                                        {"seam_rotation_order", "sequential"},
                                        {"seam_rotation_drift", -2.5},
                                        {"seam_rotation_hole_offset", "same"},
                                        {"seam_rotation_sublayers", "loop"}});
        return gcode_body(Test::slice({cyl ? make_cylinder(10., 2.) : make_cube(20., 15., 2.)}, cfg));
    };
    for (const char *pos : {"aligned", "rear", "random", "nearest"})
        for (const char *gen : {"classic", "arachne"})
            for (const bool cyl : {true, false}) {
                INFO(pos << " " << gen << (cyl ? " cylinder" : " box"));
                const std::string def = run(pos, gen, cyl, false);
                REQUIRE_FALSE(def.empty());
                CHECK(run(pos, gen, cyl, true) == def);
            }
}

TEST_CASE("Half-loop stagger on an L-shaped part", "[SeamRotating][HalfLoop][Integration]")
{
    for (const char *gen : {"arachne", "classic"}) {
        INFO(gen);
        const std::vector<Loop> loops =
            loops_by_depth(Test::slice({make_l_shape(4.)}, half_loop_config({{"perimeter_generator", gen}})), 3);
        REQUIRE(loops.size() == 4);
        for (int d = 1; d < 4; d += 2) {
            // Seam of loop d-1 projected on loop d: its arc position from loop d's own seam should be ~half a loop.
            const std::vector<Vec2d> poly = loop_polyline(loops[d]);
            const double len = polyline_length(poly);
            double arc = arc_of_nearest(poly, loops[d - 1].start);
            arc = std::min(arc, len - arc);
            INFO("depth " << d << " arc offset " << arc << " of loop length " << len);
            CHECK(std::abs(arc - len / 2.) < 8.0);
        }
    }
}

TEST_CASE("Rotating seam with sub-layers and a small scarf seam", "[SeamRotating][Integration]")
{
    const std::vector<double> ang = tagged_seam_angles(Test::slice({make_cylinder(30., 4.)}, rotating_config({
        {"layer_height", 0.3},
        {"first_layer_height", 0.3},
        {"perimeters", 3},
        {"perimeter_generator", "arachne"},
        {"external_perimeter_sublayers", 3},
        {"scarf_seam_placement", "everywhere"},
        {"scarf_seam_only_on_smooth", 0},
        {"scarf_seam_length", 2},
        {"seam_rotation_sublayers", "group"},
    })));
    const double spread[] = {0, 180, 90, 270};
    REQUIRE(ang.size() > 10);
    for (std::size_t j = 0; j < ang.size(); ++j) {
        const std::size_t k = j == 0 ? 0 : (j - 1) / 3 + 1;
        INFO("level " << j);
        // The scarf ramp displaces the recorded start by about scarf_length / r = 2 / 30 rad = 3.8 deg.
        CHECK(std::abs(wrap180(ang[j] - spread[k % 4])) < angle_tol + 4.0);
    }
}

TEST_CASE("Rotating seam with spiral vase slices without error", "[SeamRotating][Integration]")
{
    DynamicPrintConfig cfg = rotating_config({{"spiral_vase", 1}, {"perimeters", 1}, {"top_solid_layers", 0},
                                              {"bottom_solid_layers", 1}, {"fill_density", "0%"}});
    std::string gcode;
    REQUIRE_NOTHROW(gcode = Test::slice({make_cylinder(20., 6.)}, cfg));
    CHECK(gcode.find("; prusaslicer_config = begin") != std::string::npos);
}

TEST_CASE("Rotating seam config ini round trip", "[SeamRotating][Config]")
{
    DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config_with({
        {"seam_position", "rotating"},
        {"seam_rotation_define_by", "degrees"},
        {"seam_rotation_count", 7},
        {"seam_rotation_step", 12.5},
        {"seam_rotation_start", 33},
        {"seam_rotation_order", "sequential"},
        {"seam_rotation_drift", -2.5},
        {"seam_rotation_hole_offset", "same"},
        {"seam_rotation_sublayers", "loop"},
    });
    std::string ini;
    for (const std::string &key : cfg.keys())
        ini += key + " = " + cfg.opt_serialize(key) + "\n";

    DynamicPrintConfig back = DynamicPrintConfig::full_print_config();
    REQUIRE_NOTHROW(back.load_from_ini_string(ini, ForwardCompatibilitySubstitutionRule::Disable));
    CHECK(back.opt_enum<SeamPosition>("seam_position") == spRotating);
    for (const std::string &key : seam_rotation_keys()) {
        INFO(key);
        CHECK(back.opt_serialize(key) == cfg.opt_serialize(key));
    }

    const boost::filesystem::path file = boost::filesystem::temp_directory_path() /
                                         boost::filesystem::unique_path("seam-rot-%%%%-%%%%.ini");
    cfg.save(file.string());
    DynamicPrintConfig loaded = DynamicPrintConfig::full_print_config();
    REQUIRE_NOTHROW(loaded.load(file.string(), ForwardCompatibilitySubstitutionRule::Disable));
    boost::filesystem::remove(file);
    CHECK(loaded.opt_enum<SeamPosition>("seam_position") == spRotating);
    for (const std::string &key : seam_rotation_keys()) {
        INFO(key);
        CHECK(loaded.opt_serialize(key) == cfg.opt_serialize(key));
    }
}

TEST_CASE("Legacy and new keys together: the new key wins in either order", "[SeamRotating][Config][Legacy]")
{
    const auto load = [](const char *text) {
        DynamicPrintConfig cfg = DynamicPrintConfig::full_print_config();
        REQUIRE_NOTHROW(cfg.load_from_ini_string(text, ForwardCompatibilitySubstitutionRule::Disable));
        return cfg;
    };
    for (const char *text : {"external_perimeter_resolution = 3\nexternal_perimeter_sublayers = 2\n",
                             "external_perimeter_sublayers = 2\nexternal_perimeter_resolution = 3\n"})
        CHECK(load(text).opt_int("external_perimeter_sublayers") == 2);
    for (const char *text : {"swap_first_int_w_ext_perimeter = 1\nfirst_internal_perimeter_last = 0\n",
                             "first_internal_perimeter_last = 0\nswap_first_int_w_ext_perimeter = 1\n"})
        CHECK_FALSE(load(text).opt_bool("first_internal_perimeter_last"));
    for (const char *text : {"swap_first_int_w_ext_perimeter = 0\nfirst_internal_perimeter_last = 1\n",
                             "first_internal_perimeter_last = 1\nswap_first_int_w_ext_perimeter = 0\n"})
        CHECK(load(text).opt_bool("first_internal_perimeter_last"));
    // A later legacy-only load into a fresh config still maps.
    CHECK(load("external_perimeter_resolution = 4\n").opt_int("external_perimeter_sublayers") == 4);
}

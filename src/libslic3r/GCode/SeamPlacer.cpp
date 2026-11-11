///|/ Copyright (c) Prusa Research 2020 - 2023 Vojtěch Bubník @bubnikv, Lukáš Matěna @lukasmatena,
/// Pavel Mikuš @Godrak
///|/ Copyright (c) SuperSlicer 2023 Remi Durand @supermerill
///|/
///|/ PrusaSlicer is released under the terms of the AGPLv3 or higher
///|/

#include <boost/filesystem/operations.hpp>
#include <boost/property_tree/json_parser.hpp>

#include "SeamPlacer.hpp"

#include "libslic3r/ClipperUtils.hpp"
#include "libslic3r/GCode/SeamShells.hpp"
#include "libslic3r/GCode/SeamAligned.hpp"
#include "libslic3r/GCode/SeamRear.hpp"
#include "libslic3r/GCode/SeamRotating.hpp"
#include "libslic3r/GCode/SeamRandom.hpp"
#include "libslic3r/GCode/ModelVisibility.hpp"
#include "libslic3r/GCode/SeamGeometry.hpp"

namespace Slic3r::Seams {

using ObjectPainting = std::map<const PrintObject*, ModelInfo::Painting>;

ObjectLayerPerimeters get_perimeters(
    SpanOfConstPtrs<PrintObject> objects,
    const Params &params,
    const ObjectPainting& object_painting,
    const std::function<void(void)> &throw_if_canceled
) {
    ObjectLayerPerimeters result;

    for (const PrintObject *print_object : objects) {
        const ModelInfo::Painting &painting{object_painting.at(print_object)};
        throw_if_canceled();

        const std::vector<Geometry::Extrusions> extrusions{
            Geometry::get_extrusions(print_object->layers())};
        const Perimeters::LayerInfos layer_infos{Perimeters::get_layer_infos(
            print_object->layers(), params.perimeter.elephant_foot_compensation
        )};
        const std::vector<Geometry::BoundedPolygons> projected{
            Geometry::project_to_geometry(extrusions, params.max_distance)
        };
        Perimeters::LayerPerimeters perimeters{Perimeters::create_perimeters(projected, layer_infos, painting, params.perimeter)};

        throw_if_canceled();
        result.emplace(print_object, std::move(perimeters));
    }
    return result;
}

Perimeters::LayerPerimeters sort_to_layers(Shells::Shells<> &&shells) {
    const std::size_t layer_count{Shells::get_layer_count(shells)};
    Perimeters::LayerPerimeters result(layer_count);

    for (Shells::Shell<> &shell : shells) {
        for (Shells::Slice<> &slice : shell) {
            const BoundingBox bounding_box{Geometry::scaled(slice.boundary.positions)};
            result[slice.layer_index].push_back(
                Perimeters::BoundedPerimeter{std::move(slice.boundary), bounding_box}
            );
        }
    }
    return result;
}

ObjectSeams precalculate_seams(
    const Params &params,
    ObjectLayerPerimeters &&seam_data,
    const std::function<void(void)> &throw_if_canceled,
    std::map<const PrintObject *, std::vector<std::vector<std::vector<SeamChoice>>>> *pass_seams = nullptr
) {
    ObjectSeams result;

    for (auto &[print_object, layer_perimeters] : seam_data) {
        switch (print_object->config().seam_position.value) {
        case spAligned: {
            const Transform3d transformation{print_object->trafo_centered()};
            const ModelVolumePtrs &volumes{print_object->model_object()->volumes};

            Slic3r::ModelInfo::Visibility
                points_visibility{transformation, volumes, params.visibility, throw_if_canceled};
            throw_if_canceled();
            const Aligned::VisibilityCalculator visibility_calculator{
                points_visibility, params.convex_visibility_modifier,
                params.concave_visibility_modifier};

            Shells::Shells<> shells{Shells::create_shells(std::move(layer_perimeters), params.max_distance)};
            result[print_object] = Aligned::get_object_seams(
                std::move(shells), visibility_calculator, params.aligned
            );
            break;
        }
        case spRear: {
            result[print_object] = Rear::get_object_seams(std::move(layer_perimeters), params.rear_tolerance, params.rear_y_offset);
            break;
        }
        case spRotating: {
            // All rotating options are per-object: read them from the object config, never from Params.
            const Rotating::Params rotating_params{Rotating::Params::from_config(print_object->config())};
            const std::vector<std::size_t> seam_indices{Rotating::layer_seam_indices(*print_object, rotating_params)};
            std::vector<std::vector<std::optional<std::size_t>>> pass_indices;
            const bool want_table{pass_seams != nullptr && rotating_params.sublayers == srsLoop &&
                                  Rotating::has_external_sub_passes(*print_object)};
            if (want_table) {
                pass_indices = Rotating::layer_pass_seam_indices(*print_object, rotating_params);
            }
            result[print_object] = Rotating::get_object_seams(
                std::move(layer_perimeters), seam_indices, rotating_params,
                Rotating::object_frame_yaw(print_object->trafo()),
                want_table ? &pass_indices : nullptr, want_table ? &(*pass_seams)[print_object] : nullptr
            );
            break;
        }
        case spRandom: {
            result[print_object] = Random::get_object_seams(std::move(layer_perimeters), params.random_seed);
            break;
        }
        case spNearest: {
            // Do not precalculate anything.
            break;
        }
        }
        throw_if_canceled();
    }
    return result;
}

Params Placer::get_params(const DynamicPrintConfig &config) {
    Params params{};

    params.perimeter.elephant_foot_compensation = config.opt_float("elefant_foot_compensation");
    if (config.opt_int("raft_layers") > 0) {
        params.perimeter.elephant_foot_compensation = 0.0;
    }
    params.random_seed = 1653710332u;

    params.aligned.max_detour = 1.0;
    params.aligned.continuity_modifier = 2.0;
    params.convex_visibility_modifier = 1.1;
    params.concave_visibility_modifier = 0.9;
    params.perimeter.overhang_threshold = Slic3r::Geometry::deg2rad(55.0);
    params.perimeter.convex_threshold = Slic3r::Geometry::deg2rad(10.0);
    params.perimeter.concave_threshold = Slic3r::Geometry::deg2rad(15.0);


    params.max_nearest_detour = 1.0;
    params.rear_tolerance = 1.0;
    params.rear_y_offset = 20;
    params.aligned.jump_visibility_threshold = 0.6;
    params.max_distance = 5.0;
    params.perimeter.oversampling_max_distance = 0.2;
    params.perimeter.embedding_threshold = 0.5;
    params.perimeter.painting_radius = 0.05;
    params.perimeter.simplification_epsilon = 0.001;
    params.perimeter.smooth_angle_arm_length = 0.5;
    params.perimeter.sharp_angle_arm_length = 0.25;

    params.visibility.raycasting_visibility_samples_count = 30000;
    params.visibility.fast_decimation_triangle_count_target = 16000;
    params.visibility.sqr_rays_per_sample_point = 5;

    return params;
}

void Placer::init(
    SpanOfConstPtrs<PrintObject> objects,
    const Params &params,
    const std::function<void(void)> &throw_if_canceled
) {
    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: init: start";

    ObjectPainting object_painting;
    for (const PrintObject *print_object : objects) {
        const Transform3d transformation{print_object->trafo_centered()};
        const ModelVolumePtrs &volumes{print_object->model_object()->volumes};
        object_painting.emplace(print_object, ModelInfo::Painting{transformation, volumes});
    }

    ObjectLayerPerimeters perimeters{get_perimeters(objects, params, object_painting, throw_if_canceled)};
    ObjectLayerPerimeters perimeters_for_precalculation;

    for (auto &[print_object, layer_perimeters] : perimeters) {
        if (print_object->config().seam_position.value == spNearest) {
            this->perimeters_per_layer[print_object] = std::move(layer_perimeters);
        } else {
            perimeters_for_precalculation[print_object] = std::move(layer_perimeters);
        }
    }

    this->params = params;
    this->rotating_pass_seams.clear();
    this->seams_per_object = precalculate_seams(params, std::move(perimeters_for_precalculation), throw_if_canceled, &this->rotating_pass_seams);

    BOOST_LOG_TRIVIAL(debug) << "SeamPlacer: init: end";
}

const SeamPerimeterChoice &choose_closest_seam(
    const std::vector<SeamPerimeterChoice> &seams, const Polygon &loop_polygon
) {
    BoundingBoxes choose_from;
    choose_from.reserve(seams.size());
    for (const SeamPerimeterChoice &choice : seams) {
        choose_from.push_back(choice.bounding_box);
    }

    const std::size_t choice_index{
        Geometry::pick_closest_bounding_box(loop_polygon.bounding_box(), choose_from).first};

    return seams[choice_index];
}

std::pair<std::size_t, Vec2d> project_to_extrusion_loop(
    const SeamChoice &seam_choice,
    const Perimeters::Perimeter &perimeter,
    const AABBTreeLines::LinesDistancer<Linef> &distancer
) {
    const bool is_at_vertex{seam_choice.previous_index == seam_choice.next_index};
    const Vec2d edge{
        perimeter.positions[seam_choice.next_index] -
        perimeter.positions[seam_choice.previous_index]};
    const Vec2d normal{
        is_at_vertex ?
            Geometry::get_polygon_normal(perimeter.positions, seam_choice.previous_index, 0.1) :
            Geometry::get_normal(edge)};

    double depth{distancer.distance_from_lines<false>(seam_choice.position)};
    const Vec2d final_position{seam_choice.position - normal * depth};

    auto [_, loop_line_index, loop_point] = distancer.distance_from_lines_extra<false>(final_position
    );
    return {loop_line_index, loop_point};
}

double get_angle(const SeamChoice &seam_choice, const Perimeters::Perimeter &perimeter) {
    const bool is_at_vertex{seam_choice.previous_index == seam_choice.next_index};
    return is_at_vertex ? perimeter.angles[seam_choice.previous_index] : 0.0;
}

SeamChoice to_seam_choice(
    const Geometry::PointOnLine &point_on_line, const Perimeters::Perimeter &perimeter
) {
    SeamChoice result;

    result.position = point_on_line.point;
    result.previous_index = point_on_line.line_index;
    result.next_index = point_on_line.line_index == perimeter.positions.size() - 1 ?
        0 :
        point_on_line.line_index + 1;
    return result;
}

Geometry::Direction1D get_direction(
    const bool flipped,
    const ExPolygon &perimeter_polygon,
    const ExtrusionLoop &loop
) {
    using Dir = Geometry::Direction1D;

    Dir result{flipped ? Dir::forward : Dir::backward};

    // In rare cases it may happen that the original geometry perimeter
    // polygon has different direction to the actual extrusion loop.
    // In that case the logic is exactly opposite.
    if (perimeter_polygon.contour.is_clockwise() != loop.is_clockwise()) {
        result = result == Dir::forward ? Dir::backward : Dir::forward;
    }
    return result;
}

unsigned get_seam_choice_value(const SeamChoice &seam_choice, const Perimeters::Perimeter& perimeter) {
    const unsigned previous_value{Perimeters::get_point_value(
        perimeter.point_types[seam_choice.previous_index],
        perimeter.point_classifications[seam_choice.previous_index]
    )};

    const unsigned next_value{Perimeters::get_point_value(
        perimeter.point_types[seam_choice.next_index],
        perimeter.point_classifications[seam_choice.next_index]
    )};

    return std::max(previous_value, next_value);
}

// Is this loop an inner (non-external, non-overhang) perimeter? The first and second internal
// perimeter carry their own ExtrusionRole but are inner perimeters just like ExtrusionRole::Perimeter,
// so every inner-perimeter decision in this file has to accept all three.
static bool is_inner_perimeter_role(const ExtrusionRole role)
{
    return role == ExtrusionRole::Perimeter
        || role == ExtrusionRole::FirstInternalPerimeter
        || role == ExtrusionRole::SecondInternalPerimeter;
}

// Wall identity of a loop, decided from its paths collectively. Overhang/bridge role
// splitting stamps segment roles that carry no wall identity, so classification must use
// the base roles found anywhere in the loop — never ExtrusionLoop::role(), which is just
// paths.front().role() and reports OverhangPerimeter whenever the first path happens to
// cross an overhang, misclassifying inner walls as outer ones.
enum class LoopWallRole { Outer, Inner, Unknown };

static LoopWallRole loop_wall_role(const ExtrusionLoop &loop)
{
    bool has_inner{false};
    bool has_outer{false};
    for (const ExtrusionPath &path : loop.paths) {
        if (path.role() == ExtrusionRole::ExternalPerimeter)
            has_outer = true;
        else if (is_inner_perimeter_role(path.role()))
            has_inner = true;
    }
    if (has_inner)
        return LoopWallRole::Inner;
    if (has_outer)
        return LoopWallRole::Outer;
    // No base wall role left (fully overhang-split loop).
    return LoopWallRole::Unknown;
}

std::pair<std::size_t, Vec2d> point_at_arc_length(
    const Linesf &lines, const std::size_t start_line, const Vec2d &start_point, double arc
) {
    const std::size_t n{lines.size()};
    if (n == 0) {
        return {0, start_point};
    }
    double total{0.0};
    for (const Linef &line : lines) {
        total += (line.b - line.a).norm();
    }
    if (total <= EPSILON) {
        return {start_line, start_point};
    }
    arc = std::fmod(arc, total);
    if (arc < 0.0) {
        arc += total;
    }
    // Forward walk by arc (a negative request was wrapped to the equivalent forward arc).
    std::size_t index{start_line % n};
    double remaining_in_line{(lines[index].b - start_point).norm()};
    if (arc <= remaining_in_line) {
        const Vec2d dir{(lines[index].b - lines[index].a).normalized()};
        return {index, start_point + dir * arc};
    }
    arc -= remaining_in_line;
    for (std::size_t step{0}; step <= n; ++step) {
        index = (index + 1) % n;
        const double length{(lines[index].b - lines[index].a).norm()};
        if (arc <= length) {
            const Vec2d dir{length > EPSILON ? Vec2d{(lines[index].b - lines[index].a) / length} : Vec2d{Vec2d::Zero()}};
            return {index, lines[index].a + dir * arc};
        }
        arc -= length;
    }
    return {start_line, start_point};
}

namespace {
SeamChoice project_point_to_perimeter(const Vec2d &point, const Perimeters::Perimeter &perimeter) {
    const std::size_t n{perimeter.positions.size()};
    SeamChoice best{0, 0, perimeter.positions.empty() ? point : perimeter.positions[0]};
    double best_distance{std::numeric_limits<double>::infinity()};
    for (std::size_t i{0}; i < n; ++i) {
        const std::size_t j{(i + 1) % n};
        const Vec2d &a{perimeter.positions[i]};
        const Vec2d edge{perimeter.positions[j] - a};
        const double length_sq{edge.squaredNorm()};
        const double t{length_sq > EPSILON ? std::clamp((point - a).dot(edge) / length_sq, 0.0, 1.0) : 0.0};
        const Vec2d projected{a + edge * t};
        const double distance{(point - projected).squaredNorm()};
        if (distance < best_distance) {
            best_distance = distance;
            if (t <= 1e-9) {
                best = {i, i, a};
            } else if (t >= 1.0 - 1e-9) {
                best = {j, j, perimeter.positions[j]};
            } else {
                best = {i, j, projected};
            }
        }
    }
    return best;
}
} // namespace

boost::variant<Point, Scarf::Scarf> finalize_seam_position(
    const ExtrusionLoop &loop,
    const PrintRegion *region,
    SeamChoice seam_choice,
    const Perimeters::Perimeter &perimeter,
    const bool staggered_inner_seams,
    const bool half_loop,
    const bool flipped,
    const bool thick_bridges,
    const std::optional<double> min_seam_gap_rad
) {
    using Perimeters::offset_along_perimeter;
    using Perimeters::PointOnPerimeter;

    const Polygon loop_polygon{Geometry::to_polygon(loop)};
    const LoopWallRole wall_role{loop_wall_role(loop)};
    const std::size_t loop_depth{loop.paths.empty() ? 0 : loop.paths.front().attributes().perimeter_index.value_or(0)};
    // Half-loop mode: even depths keep the external seam position, odd depths are half a loop away.
    const bool do_staggering{
        staggered_inner_seams && wall_role == LoopWallRole::Inner && (!half_loop || loop_depth % 2 == 1)};
    const bool do_half_loop{do_staggering && half_loop};
    const double loop_width{loop.paths.empty() ? 0.0 : loop.paths.front().width()};

    const ExPolygon perimeter_polygon{Geometry::scaled(perimeter.positions)};
    const Linesf perimeter_lines{to_unscaled_linesf({perimeter_polygon})};
    const Linesf loop_lines{to_unscaled_linesf({ExPolygon{loop_polygon}})};
    const AABBTreeLines::LinesDistancer<Linef> distancer{loop_lines};

    auto [loop_line_index, loop_point]{
        project_to_extrusion_loop(seam_choice, perimeter, distancer)};

    const Geometry::Direction1D offset_direction{get_direction(flipped, perimeter_polygon, loop)};

    const auto offset_stop_condition{
        [choice_value = get_seam_choice_value(seam_choice, perimeter)](
            const Perimeters::Perimeter &perimeter, const std::size_t index
        ) {
            const unsigned current_point_value{Perimeters::get_point_value(
                perimeter.point_types[index], perimeter.point_classifications[index]
            )};
            if (
                current_point_value < choice_value
                && (
                    perimeter.point_types[index] == Perimeters::PointType::blocker
                    || perimeter.point_classifications[index] == Perimeters::PointClassification::overhang
                )
            ) {
                return true;
            }
            return false;
        }};

    // ExtrusionRole::Perimeter is inner perimeter.
    if (do_half_loop) {
        const double loop_length{unscaled(loop_polygon.length())};
        const auto reproject = [&](const Vec2d &point) {
            return project_point_to_perimeter(point, perimeter);
        };
        const auto acceptable = [&](const SeamChoice &choice) {
            for (const std::size_t i : {choice.previous_index, choice.next_index}) {
                if (perimeter.point_types[i] == Perimeters::PointType::blocker ||
                    perimeter.point_classifications[i] == Perimeters::PointClassification::overhang) {
                    return false;
                }
            }
            return true;
        };
        // Walk in print direction (the loop polygon is stored in print order).
        const double half{loop_length / 2.0};
        Vec2d half_point{point_at_arc_length(loop_lines, loop_line_index, loop_point, half).second};
        SeamChoice half_choice{reproject(half_point)};
        if (!acceptable(half_choice)) {
            const double step{std::max(0.2, loop_length / 200.0)};
            const int max_steps{static_cast<int>(0.1 * loop_length / step)};
            for (int k = 1; k <= max_steps; ++k) {
                bool found{false};
                for (const double sign : {1.0, -1.0}) {
                    const Vec2d candidate{
                        point_at_arc_length(loop_lines, loop_line_index, loop_point, half + sign * k * step).second};
                    const SeamChoice candidate_choice{reproject(candidate)};
                    if (acceptable(candidate_choice)) {
                        half_point = candidate;
                        half_choice = candidate_choice;
                        found = true;
                        break;
                    }
                }
                if (found) {
                    break;
                }
            }
        }
        const auto [new_line_index, new_loop_point]{project_to_extrusion_loop(half_choice, perimeter, distancer)};
        if ((new_loop_point - half_point).norm() > 5.0) {
            return scaled(half_point);
        }
        seam_choice = half_choice;
        loop_line_index = new_line_index;
        loop_point = new_loop_point;
    } else if (do_staggering) {
        const double depth = (loop_point - seam_choice.position).norm() -
            loop_width / 2.0;

        const double staggering_offset{depth};

        std::optional<PointOnPerimeter> staggered_point{offset_along_perimeter(
            {seam_choice.previous_index, seam_choice.next_index, loop_point},
            perimeter,
            staggering_offset,
            offset_direction,
            offset_stop_condition
        )};

        if (staggered_point) {
            seam_choice = *staggered_point;
            std::tie(loop_line_index, loop_point) = project_to_extrusion_loop(seam_choice, perimeter, distancer);
        }
    }

    // Scarf eligibility is decided from the loop itself, never from the matched seam-data
    // entry (which is always an external perimeter — the entry locates the seam, it does
    // not identify the loop). "contours" means the non-hole external wall only; inner walls
    // answer to scarf_seam_on_inner_perimeters in the inner branch below; loops with no
    // base wall role left (fully overhang-split) get a plain seam.
    bool place_scarf_seam {false};
    if (region->config().scarf_seam_placement == ScarfSeamPlacement::everywhere)
        place_scarf_seam = wall_role != LoopWallRole::Unknown;
    else if (region->config().scarf_seam_placement == ScarfSeamPlacement::countours)
        // "Contours" means the non-hole external wall. Hole-ness comes from the loop's own
        // polygon orientation (holes are clockwise), not from the matched seam-data entry —
        // bounding-box matching degenerates on rings, where the outer and hole bboxes nest.
        place_scarf_seam = wall_role == LoopWallRole::Outer && loop_polygon.is_counter_clockwise();
    const bool is_smooth{
        seam_choice.previous_index != seam_choice.next_index ||
        perimeter.angle_types[seam_choice.previous_index] == Perimeters::AngleType::smooth
    };

    if (perimeter.layer_index == 0) {
        place_scarf_seam = false;
    }

    if (region->config().scarf_seam_only_on_smooth && !is_smooth) {
        place_scarf_seam = false;
    }

    if (region->config().scarf_seam_length.value <= std::numeric_limits<double>::epsilon()) {
        place_scarf_seam = false;
    }

    const bool is_overhang{
        perimeter.point_classifications[seam_choice.previous_index] == Perimeters::PointClassification::overhang
        || perimeter.point_classifications[seam_choice.next_index] == Perimeters::PointClassification::overhang
    };

    if (thick_bridges && is_overhang) {
        place_scarf_seam = false;
    }

    // Rotating seams: a scarf longer than half the smallest seam gap would overlap the neighbouring seams.
    // Decided on the matched external perimeter (not on this loop's own length) so that every loop of
    // the perimeter group gets the same verdict.
    if (place_scarf_seam && min_seam_gap_rad && !region->config().scarf_seam_entire_loop) {
        double perimeter_length{0.0};
        const std::size_t count{perimeter.positions.size()};
        for (std::size_t i{0}; i < count; ++i) {
            perimeter_length += (perimeter.positions[(i + 1) % count] - perimeter.positions[i]).norm();
        }
        if (Rotating::scarf_guard_blocks(region->config().scarf_seam_length.value, perimeter_length, *min_seam_gap_rad)) {
            return scaled(loop_point);
        }
    }

    if (place_scarf_seam) {
        Scarf::Scarf scarf{};
        scarf.entire_loop = region->config().scarf_seam_entire_loop;
        scarf.max_segment_length = region->config().scarf_seam_max_segment_length;
        scarf.start_height = std::min(region->config().scarf_seam_start_height.get_abs_value(1.0), 1.0);

        const double offset{scarf.entire_loop ? 0.0 : region->config().scarf_seam_length.value};
        const std::optional<PointOnPerimeter> outter_scarf_start_point{offset_along_perimeter(
            seam_choice,
            perimeter,
            offset,
            offset_direction,
            offset_stop_condition
        )};
        if (!outter_scarf_start_point) {
            return scaled(loop_point);
        }

        if (wall_role == LoopWallRole::Outer) { // Outer perimeter
            const Vec2d start_point_candidate{project_to_extrusion_loop(
                *outter_scarf_start_point,
                perimeter,
                distancer
            ).second};
            if ((start_point_candidate - outter_scarf_start_point->position).norm() > 5.0) {
                return scaled(loop_point);
            }
            scarf.start_point = scaled(start_point_candidate);
            scarf.end_point = scaled(loop_point);
            scarf.end_point_previous_index = loop_line_index;
            return scarf;
        } else if (wall_role == LoopWallRole::Inner) {
            PointOnPerimeter inner_scarf_end_point{
                *outter_scarf_start_point
            };

            if (region->config().external_perimeters_first.value) {
                const auto external_first_offset_direction{
                    offset_direction == Geometry::Direction1D::forward ?
                    Geometry::Direction1D::backward :
                    Geometry::Direction1D::forward
                };
                if (auto result{offset_along_perimeter(
                    seam_choice,
                    perimeter,
                    offset,
                    external_first_offset_direction,
                    offset_stop_condition
                )}) {
                    inner_scarf_end_point = *result;
                } else {
                    return scaled(seam_choice.position);
                }
            }

            if (!region->config().scarf_seam_on_inner_perimeters) {
                return scaled(inner_scarf_end_point.position);
            }

            const std::optional<PointOnPerimeter> inner_scarf_start_point{offset_along_perimeter(
                inner_scarf_end_point,
                perimeter,
                offset,
                offset_direction,
                offset_stop_condition
            )};

            if (!inner_scarf_start_point) {
                return scaled(inner_scarf_end_point.position);
            }
            const Vec2d start_point_candidate{project_to_extrusion_loop(
                *inner_scarf_start_point,
                perimeter,
                distancer
            ).second};
            if ((start_point_candidate - inner_scarf_start_point->position).norm() > 5.0) {
                return scaled(loop_point);
            }
            scarf.start_point = scaled(start_point_candidate);

            const auto [end_point_previous_index, end_point]{project_to_extrusion_loop(
                inner_scarf_end_point,
                perimeter,
                distancer
            )};
            if ((end_point - inner_scarf_end_point.position).norm() > 5.0) {
                return scaled(loop_point);
            }

            scarf.end_point = scaled(end_point);
            scarf.end_point_previous_index = end_point_previous_index;
            return scarf;
        }
    }

    return scaled(loop_point);
}

struct NearestCorner {
    Vec2d prefered_position;

    std::optional<SeamChoice> operator()(
        const Perimeters::Perimeter &perimeter,
        const Perimeters::PointType point_type,
        const Perimeters::PointClassification point_classification
    ) const {
        std::optional<SeamChoice> corner_candidate;
        double min_distance{std::numeric_limits<double>::infinity()};
        for (std::size_t i{0}; i < perimeter.positions.size(); ++i) {
            if (perimeter.point_types[i] == point_type &&
                perimeter.point_classifications[i] == point_classification &&
                perimeter.angle_types[i] != Perimeters::AngleType::smooth) {
                const Vec2d &point{perimeter.positions[i]};
                const double distance{(prefered_position - point).norm()};
                if (!corner_candidate || distance < min_distance) {
                    corner_candidate = {i, i, point};
                    min_distance = distance;
                }
            }
        }
        return corner_candidate;
    }
};

std::pair<SeamChoice, std::size_t> place_seam_near(
    const std::vector<Perimeters::BoundedPerimeter> &layer_perimeters,
    const ExtrusionLoop &loop,
    const Point &position,
    const double max_detour
) {
    BoundingBoxes choose_from;
    choose_from.reserve(layer_perimeters.size());
    for (const Perimeters::BoundedPerimeter &perimeter : layer_perimeters) {
        choose_from.push_back(perimeter.bounding_box);
    }

    const Polygon loop_polygon{Geometry::to_polygon(loop)};

    const std::size_t choice_index{
        Geometry::pick_closest_bounding_box(loop_polygon.bounding_box(), choose_from).first};

    const NearestCorner nearest_corner{unscaled(position)};
    const std::optional<SeamChoice> corner_choice{
        Seams::maybe_choose_seam_point(layer_perimeters[choice_index].perimeter, nearest_corner)};

    if (corner_choice) {
        return {*corner_choice, choice_index};
    }

    const Seams::Aligned::Impl::Nearest nearest{unscaled(position), max_detour};
    const SeamChoice nearest_choice{
        Seams::choose_seam_point(layer_perimeters[choice_index].perimeter, nearest)};

    return {nearest_choice, choice_index};
}

int get_perimeter_count(const Layer *layer){
    int count{0};
    for (const LayerRegion *layer_region : layer->regions()) {
        // On the sub-layers of a perimeter combine group the internal-perimeter entities were
        // moved to the group-top layer, so the live inventory no longer reflects the layer's
        // real perimeter structure. Use the stashed pre-combine count instead, so seam
        // placement (e.g. the 2-or-3-perimeter special case) behaves identically on every
        // layer of a combine group.
        if (layer_region->perimeters_moved_to_upper_layer() &&
            layer_region->perimeter_entity_count_pre_combine() >= 0) {
            count += layer_region->perimeter_entity_count_pre_combine();
            continue;
        }
        for (const ExtrusionEntity *ex_entity : layer_region->perimeters()) {
            if (ex_entity->is_collection()) { //collection of inner, outer, and overhang perimeters
                count += static_cast<const ExtrusionEntityCollection*>(ex_entity)->entities.size();
            }
            else {
                // Skip two-pass bridge foundation copies — they are not real perimeter contours.
                if (const auto *path = dynamic_cast<const ExtrusionPath *>(ex_entity);
                        path && path->attributes().pass_index.has_value()
                        && *path->attributes().pass_index == 0)
                    continue;
                count += 1;
            }
        }
    }
    return count;
}

boost::variant<Point, Scarf::Scarf> Placer::place_seam(
    const Layer *layer, const PrintRegion *region, const ExtrusionLoop &loop, const bool flipped, const Point &last_pos,
    std::optional<std::size_t> sub_pass
) const {
    const PrintObject *po = layer->object();
    // Must not be called with supprot layer.
    assert(dynamic_cast<const SupportLayer *>(layer) == nullptr);
    // Object layer IDs are incremented by the number of raft layers.
    assert(layer->id() >= po->slicing_parameters().raft_layers());
    const size_t layer_index = layer->id() - po->slicing_parameters().raft_layers();

    const bool thick_bridges{po->config().thick_bridges.value};
    const bool staggered_inner_seams{po->config().staggered_inner_seams.value};
    const bool half_loop{po->config().staggered_inner_seams_mode.value == sismHalfLoop};
    std::optional<double> min_gap;
    if (po->config().seam_position.value == spRotating) {
        min_gap = Rotating::min_seam_gap_rad(Rotating::Params::from_config(po->config()));
    }

    if (po->config().seam_position.value == spNearest) {
        const std::vector<Perimeters::BoundedPerimeter> &perimeters{
            this->perimeters_per_layer.at(po)[layer_index]};
        const auto [seam_choice, perimeter_index] =
            place_seam_near(perimeters, loop, last_pos, this->params.max_nearest_detour);
        return finalize_seam_position(
            loop, region, seam_choice, perimeters[perimeter_index].perimeter,
            staggered_inner_seams, half_loop, flipped, thick_bridges, min_gap
        );
    } else {
        const std::vector<SeamPerimeterChoice> &seams_on_perimeters{this->seams_per_object.at(po)[layer_index]};
        // Rotating loop mode: per-sub-pass seam choices, parallel to seams_on_perimeters.
        const std::vector<SeamChoice> *pass_choices{nullptr};
        if (sub_pass.has_value()) {
            if (const auto table = this->rotating_pass_seams.find(po); table != this->rotating_pass_seams.end() &&
                layer_index < table->second.size() && *sub_pass < table->second[layer_index].size() &&
                table->second[layer_index][*sub_pass].size() == seams_on_perimeters.size()) {
                pass_choices = &table->second[layer_index][*sub_pass];
            }
        }
        const auto choice_of = [&](const SeamPerimeterChoice &entry) -> const SeamChoice & {
            return pass_choices != nullptr ? (*pass_choices)[&entry - seams_on_perimeters.data()] : entry.choice;
        };

        // Special case.
        // If there are only two perimeters and the current perimeter is hole (clockwise).
        const Polygon loop_polygon{Geometry::to_polygon(loop)};
        const LoopWallRole wall_role{loop_wall_role(loop)};
        const SeamPerimeterChoice &closest_seam{choose_closest_seam(seams_on_perimeters, loop_polygon)};

        const int perimeter_count{get_perimeter_count(layer)};
        const bool has_2_or_3_perimeters{perimeter_count == 2 || perimeter_count == 3};
        // The 2-or-3 special case exists for the degenerate ring case, where the outer and
        // hole bounding boxes nest and bbox matching is unreliable. It may only override the
        // matching for the loop that IS the non-hole external wall — decided from the loop
        // itself (base wall role + polygon orientation), never from the matching. Feeding
        // that entry to any other loop (holes, internal walls) locates the seam on foreign
        // geometry and targets the scarf joint at the wrong loop.
        if (has_2_or_3_perimeters &&
            seams_on_perimeters.size() == 2 &&
            seams_on_perimeters[0].perimeter.is_hole != seams_on_perimeters[1].perimeter.is_hole &&
            wall_role == LoopWallRole::Outer && loop_polygon.is_counter_clockwise()) {
            const SeamPerimeterChoice &non_hole_seam{
                seams_on_perimeters[0].perimeter.is_hole ? seams_on_perimeters[1] :
                                                           seams_on_perimeters[0]};
            return finalize_seam_position(
                loop, region, choice_of(non_hole_seam), non_hole_seam.perimeter,
                staggered_inner_seams, half_loop, flipped, thick_bridges, min_gap
            );
        }

        return finalize_seam_position(
            loop, region, choice_of(closest_seam), closest_seam.perimeter,
            staggered_inner_seams, half_loop, flipped, thick_bridges, min_gap
        );
    }
}
} // namespace Slic3r::Seams

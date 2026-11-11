#include "libslic3r/GCode/SeamRotating.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <optional>
#include <utility>

#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/GCode/SeamAligned.hpp"
#include "libslic3r/GCode/SeamChoice.hpp"
#include "libslic3r/GCode/SeamPerimeters.hpp"
#include "libslic3r/Geometry.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Print.hpp"

namespace Slic3r::Seams::Rotating {

namespace Impl {
constexpr std::size_t max_table_n{36};
constexpr std::size_t max_exhaustive_n{12};

std::size_t cyclic_gap(const std::size_t a, const std::size_t b, const std::size_t n) {
    const std::size_t d{a > b ? a - b : b - a};
    return std::min(d, n - d);
}

// Largest stride s <= n/2 coprime to n: k*s mod n has all consecutive gaps equal to s.
std::size_t best_stride(const std::size_t n) {
    for (std::size_t s{n / 2}; s >= 1; --s) {
        if (std::gcd(s, n) == 1) {
            return s;
        }
    }
    return 1;
}

std::vector<std::size_t> stride_order(const std::size_t n) {
    const std::size_t s{best_stride(n)};
    std::vector<std::size_t> result(n);
    for (std::size_t k{0}; k < n; ++k) {
        result[k] = (k * s) % n;
    }
    return result;
}

struct Search
{
    std::size_t n;
    std::size_t best_min;
    bool has_best{false};
    std::vector<std::size_t> best_sorted_gaps;
    std::vector<std::size_t> best_sequence;
    std::vector<std::size_t> sequence;
    std::vector<bool> used;

    void evaluate() {
        std::vector<std::size_t> gaps;
        for (std::size_t i{0}; i < n; ++i) {
            gaps.push_back(cyclic_gap(sequence[i], sequence[(i + 1) % n], n));
        }
        std::sort(gaps.begin(), gaps.end());
        if (gaps.front() < best_min) {
            return;
        }
        // Larger sorted gap vector wins; then the larger final element (this is what yields the required
        // 0,2,1,3 / 0,2,5,3,1,4 among otherwise tied sequences); on equality keep the earlier
        // (lowest lexicographic) sequence.
        const bool better{
            !has_best || gaps > best_sorted_gaps ||
            (gaps == best_sorted_gaps && sequence.back() > best_sequence.back())};
        if (better) {
            has_best = true;
            best_min = gaps.front();
            best_sorted_gaps = std::move(gaps);
            best_sequence = sequence;
        }
    }

    void run() {
        if (sequence.size() == n) {
            evaluate();
            return;
        }
        for (std::size_t c{1}; c < n; ++c) {
            if (used[c] || cyclic_gap(sequence.back(), c, n) < best_min) {
                continue;
            }
            used[c] = true;
            sequence.push_back(c);
            run();
            sequence.pop_back();
            used[c] = false;
        }
    }
};

std::vector<std::size_t> compute_exhaustive(const std::size_t n) {
    if (n <= 2) {
        std::vector<std::size_t> result(n);
        std::iota(result.begin(), result.end(), 0);
        return result;
    }
    // The stride order is a valid lower bound for the optimal minimum gap, so pruning below it is safe.
    Search search{n, best_stride(n), false, {}, {}, {0}, std::vector<bool>(n, false)};
    search.used[0] = true;
    search.run();
    return search.best_sequence;
}

std::vector<std::vector<std::size_t>> build_table() {
    std::vector<std::vector<std::size_t>> table(max_table_n + 1);
    for (std::size_t n{1}; n <= max_table_n; ++n) {
        table[n] = n <= max_exhaustive_n ? compute_exhaustive(n) : stride_order(n);
    }
    return table;
}
} // namespace Impl

Params Params::from_config(const ConfigBase &config) {
    Params p;
    p.define_by = config.opt_enum<SeamRotationDefineBy>("seam_rotation_define_by");
    p.count = std::clamp(config.opt_int("seam_rotation_count"), 1, 36);
    p.step_deg = config.opt_float("seam_rotation_step");
    p.start_deg = config.opt_float("seam_rotation_start");
    p.order = config.opt_enum<SeamRotationOrder>("seam_rotation_order");
    p.drift_deg = config.opt_float("seam_rotation_drift");
    p.hole_opposite = config.opt_enum<SeamRotationHoleOffset>("seam_rotation_hole_offset") == srhoOpposite;
    p.sublayers = config.opt_enum<SeamRotationSublayers>("seam_rotation_sublayers");
    return p;
}

const std::vector<std::size_t> &spread_order(const std::size_t n) {
    static const std::vector<std::vector<std::size_t>> table{Impl::build_table()};
    static const std::vector<std::size_t> empty;
    static thread_local std::vector<std::size_t> large; // N above the table (not reachable from config).
    if (n == 0) {
        return empty;
    }
    if (n <= Impl::max_table_n) {
        return table[n];
    }
    large = Impl::stride_order(n);
    return large;
}

std::vector<std::size_t> sequential_order(const std::size_t n) {
    std::vector<std::size_t> result(n);
    std::iota(result.begin(), result.end(), 0);
    return result;
}

double rotation_angle_deg(const std::size_t k, const Params &params) {
    const double kd{static_cast<double>(k)};
    if (params.define_by == srdDegrees) {
        // Degrees mode is always sequential.
        return params.start_deg + (params.step_deg + params.drift_deg) * kd;
    }
    const std::size_t n{static_cast<std::size_t>(std::max(1, params.count))};
    const std::size_t position{
        params.order == sroSpread ? spread_order(n)[k % n] : k % n};
    return params.start_deg + static_cast<double>(position) * 360.0 / static_cast<double>(n)
        + params.drift_deg * kd;
}

std::size_t effective_positions(const Params &params) {
    if (params.define_by == srdCount) {
        return static_cast<std::size_t>(std::max(1, params.count));
    }
    if (std::abs(params.drift_deg) > 1e-9) {
        double gap{std::fmod(std::abs(params.step_deg + params.drift_deg), 360.0)};
        if (gap > 180.0) {
            gap = 360.0 - gap;
        }
        if (gap < 1e-3) {
            return 360;
        }
        return std::min<std::size_t>(360, static_cast<std::size_t>(std::llround(360.0 / gap)));
    }
    // Work in hundredths of a degree so that the gcd is exact.
    const long long step100{std::llround(std::fmod(std::abs(params.step_deg), 360.0) * 100.0)};
    if (step100 == 0) {
        return 1;
    }
    const long long g{std::gcd(step100, 36000LL)};
    return std::min<std::size_t>(360, static_cast<std::size_t>(36000 / g));
}

double min_seam_gap_rad(const Params &params) {
    constexpr double two_pi{2.0 * M_PI};
    if (params.define_by == srdCount) {
        const std::size_t n{static_cast<std::size_t>(std::max(1, params.count))};
        const std::vector<std::size_t> order{params.order == sroSpread ? spread_order(n) : sequential_order(n)};
        // Consecutive deltas include the per-layer drift, folded to [0, 180] degrees.
        double best_deg{360.0};
        for (std::size_t k = 0; k < n; ++k) {
            const double delta{
                (static_cast<double>(order[(k + 1) % n]) - static_cast<double>(order[k])) * 360.0 /
                    static_cast<double>(n) + params.drift_deg};
            double d{std::fmod(std::abs(delta), 360.0)};
            if (d > 180.0) {
                d = 360.0 - d;
            }
            best_deg = std::min(best_deg, d);
        }
        if (best_deg < 1e-3) {
            return two_pi;
        }
        return best_deg * M_PI / 180.0;
    }
    double gap{std::fmod(std::abs(params.step_deg + params.drift_deg), 360.0)};
    if (gap > 180.0) {
        gap = 360.0 - gap;
    }
    if (gap < 1e-3) {
        return two_pi;
    }
    return gap * M_PI / 180.0;
}

bool scarf_guard_blocks(const double scarf_length, const double perimeter_length, const double min_gap_rad) {
    return scarf_length > perimeter_length * min_gap_rad / (2.0 * M_PI) * 0.5;
}

ObjectFrame object_frame_yaw(const Transform3d &trafo) {
    const Eigen::Matrix2d xy{trafo.matrix().block<2, 2>(0, 0)};
    return ObjectFrame{std::atan2(xy(1, 0), xy(0, 0)), xy.determinant() < 0.0};
}

double to_bed_angle_rad(const double object_angle_deg, const ObjectFrame &frame) {
    const double theta{Slic3r::Geometry::deg2rad(object_angle_deg)};
    // Mirrored: CCW in the object frame stays CCW on the part, i.e. clockwise in bed coordinates.
    return frame.mirrored ? frame.yaw - theta : frame.yaw + theta;
}

static bool sub_pass_present(const Layer *layer, std::size_t pass) {
    if (layer == nullptr || layer->external_resolution() <= 1 || pass + 1 >= layer->external_resolution()) {
        return false;
    }
    for (const LayerRegion *layerm : layer->regions()) {
        const std::vector<ExtrusionEntityCollection> &passes = layerm->external_sub_passes();
        if (pass < passes.size() && !passes[pass].empty()) {
            return true;
        }
    }
    return false;
}

bool has_external_sub_passes(const PrintObject &print_object) {
    for (const Layer *layer : print_object.layers()) {
        if (layer->external_resolution() > 1) {
            return true;
        }
    }
    return false;
}

std::vector<std::vector<std::optional<std::size_t>>> layer_pass_seam_indices(
    const PrintObject &print_object, const Params &params
) {
    const auto &layers = print_object.layers();
    std::vector<std::vector<std::optional<std::size_t>>> result;
    result.reserve(layers.size());
    std::size_t counter{0};
    for (std::size_t i{0}; i < layers.size(); ++i) {
        const Layer *layer{layers[i]};
        const std::size_t k{std::max<std::size_t>(1, layer->external_resolution())};
        std::vector<std::optional<std::size_t>> passes(k);
        if (params.sublayers == srsLoop) {
            // Lower passes first, main pass last; every present virtual pass advances the index.
            for (std::size_t j{0}; j + 1 < k; ++j) {
                if (sub_pass_present(layer, j)) {
                    passes[j] = counter++;
                }
            }
            passes[k - 1] = counter++;
        } else {
            for (auto &pass : passes) {
                pass = counter;
            }
            if (!layer->is_pure_perimeter_sub_layer()) {
                ++counter;
            }
        }
        result.push_back(std::move(passes));
    }
    return result;
}

std::vector<std::size_t> layer_seam_indices(const PrintObject &print_object, const Params &params) {
    std::vector<std::size_t> result;
    for (const auto &passes : layer_pass_seam_indices(print_object, params)) {
        // The main pass is the last one.
        result.push_back(*passes.back());
    }
    return result;
}

static std::vector<SeamChoice> layer_seam_choices(
    const std::vector<Perimeters::BoundedPerimeter> &layer, const double theta, const Params &params
) {
    std::vector<SeamChoice> result_layer;
    for (const Perimeters::BoundedPerimeter &perimeter : layer) {
        if (perimeter.perimeter.is_degenerate) {
            std::optional<Seams::SeamChoice> seam_choice{
                Seams::choose_degenerate_seam_point(perimeter.perimeter)};
            result_layer.push_back(seam_choice ? *seam_choice : SeamChoice{});
            continue;
        }

        const BoundingBoxf bounding_box{unscaled(perimeter.bounding_box)};
        const Vec2d center{(bounding_box.min + bounding_box.max) / 2.0};
        const double half_diagonal{bounding_box.size().norm() / 2.0};

        double angle{theta};
        if (perimeter.perimeter.is_hole && params.hole_opposite) {
            angle += M_PI;
        }
        const Vec2d direction{std::cos(angle), std::sin(angle)};
        // Far outside the perimeter: the nearest point on it is where the ray through the centre exits.
        const Vec2d preferred{center + direction * (half_diagonal + 20.0)};

        const std::vector<Vec2d> &positions{perimeter.perimeter.positions};
        double length{0.0};
        for (std::size_t i{0}; i < positions.size(); ++i) {
            length += (positions[(i + 1) % positions.size()] - positions[i]).norm();
        }
        const double max_detour{std::min(1.0, 0.05 * length)};

        result_layer.push_back(Seams::choose_seam_point(
            perimeter.perimeter, Aligned::Impl::Nearest{preferred, max_detour}
        ));
    }
    return result_layer;
}

std::vector<std::vector<SeamPerimeterChoice>> get_object_seams(
    std::vector<std::vector<Perimeters::BoundedPerimeter>> &&perimeters,
    const std::vector<std::size_t> &seam_index,
    const Params &params,
    const ObjectFrame &frame,
    const std::vector<std::vector<std::optional<std::size_t>>> *pass_seam_index,
    std::vector<std::vector<std::vector<SeamChoice>>> *pass_choices
) {
    std::vector<std::vector<SeamPerimeterChoice>> result;
    if (pass_choices != nullptr) {
        pass_choices->clear();
    }
    for (std::size_t layer_index{0}; layer_index < perimeters.size(); ++layer_index) {
        std::vector<Perimeters::BoundedPerimeter> &layer{perimeters[layer_index]};
        const std::size_t k{layer_index < seam_index.size() ? seam_index[layer_index] : layer_index};
        const double theta{to_bed_angle_rad(rotation_angle_deg(k, params), frame)};
        std::vector<SeamChoice> choices{layer_seam_choices(layer, theta, params)};

        if (pass_choices != nullptr && pass_seam_index != nullptr) {
            pass_choices->emplace_back();
            if (layer_index < pass_seam_index->size()) {
                for (const std::optional<std::size_t> &pass_k : (*pass_seam_index)[layer_index]) {
                    if (!pass_k) {
                        pass_choices->back().emplace_back();
                    } else if (*pass_k == k) {
                        pass_choices->back().push_back(choices);
                    } else {
                        const double pass_theta{to_bed_angle_rad(rotation_angle_deg(*pass_k, params), frame)};
                        pass_choices->back().push_back(layer_seam_choices(layer, pass_theta, params));
                    }
                }
            }
        }

        result.emplace_back();
        for (std::size_t i{0}; i < layer.size(); ++i) {
            result.back().push_back(SeamPerimeterChoice{choices[i], std::move(layer[i].perimeter)});
        }
    }
    return result;
}
} // namespace Slic3r::Seams::Rotating

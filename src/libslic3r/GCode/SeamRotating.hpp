#ifndef libslic3r_SeamRotating_hpp_
#define libslic3r_SeamRotating_hpp_

#include <cstddef>
#include <optional>
#include <vector>

#include "libslic3r/GCode/SeamPerimeters.hpp"
#include "libslic3r/GCode/SeamChoice.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PrintConfig.hpp"

namespace Slic3r {
class PrintObject;
namespace Seams {
struct SeamPerimeterChoice;
}  // namespace Seams
}  // namespace Slic3r

namespace Slic3r::Seams::Rotating {

// All values come from the per-object config (never from Seams::Params).
struct Params
{
    SeamRotationDefineBy define_by{srdCount};
    int count{4};
    double step_deg{90.0};
    double start_deg{0.0};
    SeamRotationOrder order{sroSpread};
    double drift_deg{0.0};
    bool hole_opposite{true};
    SeamRotationSublayers sublayers{srsGroup};

    static Params from_config(const ConfigBase &config);
};

// Yaw (rad) of the object frame in bed coordinates and whether the XY block mirrors.
struct ObjectFrame
{
    double yaw{0.0};
    bool mirrored{false};
};

// Permutation of 0..N-1 maximising the minimum cyclic consecutive gap (wrap counts).
const std::vector<std::size_t> &spread_order(std::size_t n);
std::vector<std::size_t> sequential_order(std::size_t n);

// Target angle (degrees, object frame, not normalised) for seam layer index k.
double rotation_angle_deg(std::size_t k, const Params &params);

// Number of distinct positions the pattern visits (used by the scarf guard).
std::size_t effective_positions(const Params &params);

// Smallest angular gap (rad, in (0, 2*pi]) between consecutive seam angles the pattern actually uses.
double min_seam_gap_rad(const Params &params);

// True if a (non entire-loop) scarf of scarf_length would span more than half of the smallest
// seam gap on a loop of perimeter_length (same units as scarf_length).
bool scarf_guard_blocks(double scarf_length, double perimeter_length, double min_gap_rad);

ObjectFrame object_frame_yaw(const Transform3d &trafo);

// Object-frame angle (degrees) to bed angle (radians).
double to_bed_angle_rad(double object_angle_deg, const ObjectFrame &frame);

// Seam layer index per object layer (group or loop counting, spec 1.5).
std::vector<std::size_t> layer_seam_indices(const PrintObject &print_object, const Params &params);

bool has_external_sub_passes(const PrintObject &print_object);

// [layer][pass] seam index of every present virtual pass in print order (nullopt = pass absent).
// Loop mode: lower passes first, the main pass (last) gets the layer's layer_seam_indices() value.
std::vector<std::vector<std::optional<std::size_t>>> layer_pass_seam_indices(
    const PrintObject &print_object, const Params &params
);

std::vector<std::vector<SeamPerimeterChoice>> get_object_seams(
    std::vector<std::vector<Perimeters::BoundedPerimeter>> &&perimeters,
    const std::vector<std::size_t> &seam_index,
    const Params &params,
    const ObjectFrame &frame,
    // Optional: also fill pass_choices[layer][pass][perimeter] (same perimeter order as the result; absent
    // passes empty) for every pass listed in pass_seam_index.
    const std::vector<std::vector<std::optional<std::size_t>>> *pass_seam_index = nullptr,
    std::vector<std::vector<std::vector<SeamChoice>>> *pass_choices = nullptr
);
} // namespace Slic3r::Seams::Rotating

#endif // libslic3r_SeamRotating_hpp_

#ifndef slic3r_GCode_PerimeterOrder_hpp_
#define slic3r_GCode_PerimeterOrder_hpp_

#include <stddef.h>
#include <limits>
#include <vector>
#include <cstddef>

#include "libslic3r/Arachne/utils/ExtrusionLine.hpp"
#include "libslic3r/BoundingBox.hpp"
#include "libslic3r/Polygon.hpp"

namespace Slic3r::Arachne::PerimeterOrder {

// Data structure stores ExtrusionLine (closed and open) together with additional data.
struct PerimeterExtrusion
{
    explicit PerimeterExtrusion(const Arachne::ExtrusionLine &extrusion, const double area, const Polygon &polygon, const BoundingBox &bbox)
        : extrusion(extrusion), area(area), polygon(polygon), bbox(bbox) {}

    Arachne::ExtrusionLine             extrusion;
    // Absolute value of the area of the polygon. The value is always non-negative, even for holes.
    double                             area = 0;

    // Polygon is non-empty only for closed extrusions.
    Polygon                            polygon;
    BoundingBox                        bbox;

    std::vector<PerimeterExtrusion *>  adjacent_perimeter_extrusions;

    // How far is this perimeter from the nearest external perimeter. Contour is always preferred over holes.
    size_t                             depth                      = std::numeric_limits<size_t>::max();
    PerimeterExtrusion                *nearest_external_perimeter = nullptr;

    // Returns if ExtrusionLine is a contour or a hole.
    bool is_contour() const { return extrusion.is_contour(); }

    // Returns if ExtrusionLine is closed or opened.
    bool is_closed() const { return extrusion.is_closed; }

    // Returns if ExtrusionLine is an external or an internal perimeter.
    bool is_external_perimeter() const { return extrusion.is_external_perimeter(); }
    bool is_first_internal_perimeter() const { return extrusion.is_first_internal_perimeter(); }
    bool is_second_internal_perimeter() const { return extrusion.is_second_internal_perimeter(); }
};


struct PerimeterDepthLayer {
    std::vector<const PerimeterExtrusion*> externals;       // All depth 0
    std::vector<const PerimeterExtrusion*> first_internals; // All depth 1
    std::vector<const PerimeterExtrusion*> second_internals;// All depth 2
    std::vector<const PerimeterExtrusion*> other_internals; // depth 3+

    bool has_groove_structure() const {
        return !externals.empty() &&
        !first_internals.empty() &&
        !second_internals.empty();
    }

    bool is_simple_two_perimeter() const {
        return externals.size() == 1 &&
        first_internals.size() == 1 &&
        second_internals.empty();
    }

    size_t total_perimeter_count() const {
        return externals.size() + first_internals.size() +
        second_internals.size() + other_internals.size();
    }
};

// Pattern types for multiple group interactions
enum class MultiGroupPattern {
    INDEPENDENT,      // Groups don't interact (separate islands)
    CONCENTRIC,       // Nested groups (contour + holes)
    ADJACENT,         // Groups touch/share boundaries
    COMPLEX           // Mixed patterns
};

using PerimeterExtrusions = std::vector<PerimeterExtrusion>;

PerimeterExtrusions ordered_perimeter_extrusions(const Perimeters &perimeters, bool external_perimeters_first,  bool enable_injection_molding_order,  bool reverse_internal_perimeters, int reverse_internal_perimeters_at);

} // namespace Slic3r::Arachne::PerimeterOrder

#endif // slic3r_GCode_PerimeterOrder_hpp_
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
        : inset_idx(extrusion.inset_idx), extrusion(extrusion), area(area), polygon(polygon), bbox(bbox) {}

    // Physical inset index from the outside: 0 = external perimeter, 1 = first internal,
    // 2 = second internal, etc. Copied directly from ExtrusionLine::inset_idx at construction,
    // which is the authoritative source set by Arachne's skeleton algorithm.
    // Use this (not depth) for any decision that must match the FirstInternalPerimeter /
    // SecondInternalPerimeter role assignments made in PerimeterGenerator.cpp.
    size_t                             inset_idx                  = std::numeric_limits<size_t>::max();

    Arachne::ExtrusionLine             extrusion;
    // Absolute value of the area of the polygon. The value is always non-negative, even for holes.
    double                             area = 0;

    // Polygon is non-empty only for closed extrusions.
    Polygon                            polygon;
    BoundingBox                        bbox;

    std::vector<PerimeterExtrusion *>  adjacent_perimeter_extrusions;

    // Graph distance from the nearest external perimeter (BFS depth). Contour is always preferred
    // over holes. In simple, non-branching regions depth == inset_idx. In thin-wall or branching
    // regions they can diverge: depth reflects graph topology while inset_idx reflects the physical
    // wall slot. Prefer inset_idx for role-based decisions; keep depth for graph traversal only.
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

using PerimeterExtrusions = std::vector<PerimeterExtrusion>;

// swap_first_int_w_ext_perimeter: move the first internal perimeter of each group to print last within that group ("groove injection").
// reverse_internal_perimeters: within each group, reverse the print order of internal perimeters at depth >= reverse_internal_perimeters_at.
PerimeterExtrusions ordered_perimeter_extrusions(
    const Perimeters &perimeters,
    bool external_perimeters_first,
    bool swap_first_int_w_ext_perimeter = false,
    bool reverse_internal_perimeters = false,
    int  reverse_internal_perimeters_at = 0);

} // namespace Slic3r::Arachne::PerimeterOrder

#endif // slic3r_GCode_Travels_hpp_

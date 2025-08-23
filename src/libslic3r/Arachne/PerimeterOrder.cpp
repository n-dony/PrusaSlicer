#include <stack>
#include <algorithm>
#include <cmath>

#include "PerimeterOrder.hpp"
#include "libslic3r/Arachne/utils/ExtrusionJunction.hpp"
#include "libslic3r/Point.hpp"

namespace Slic3r::Arachne::PerimeterOrder {

using namespace Arachne;

struct GroupedPerimeterExtrusions
{
    GroupedPerimeterExtrusions() = delete;
    explicit GroupedPerimeterExtrusions(const PerimeterExtrusion *external_perimeter_extrusion)
        : external_perimeter_extrusion(external_perimeter_extrusion) {}

    std::vector<const PerimeterExtrusion *> extrusions;
    const PerimeterExtrusion               *external_perimeter_extrusion  = nullptr;
};

static size_t get_extrusion_lines_count(const Perimeters &perimeters) {
    size_t extrusion_lines_count = 0;
    for (const Perimeter &perimeter : perimeters)
        extrusion_lines_count += perimeter.size();

    return extrusion_lines_count;
}

static PerimeterExtrusions get_sorted_perimeter_extrusions_by_area(const Perimeters &perimeters) {
    PerimeterExtrusions sorted_perimeter_extrusions;
    sorted_perimeter_extrusions.reserve(get_extrusion_lines_count(perimeters));

    for (const Perimeter &perimeter : perimeters) {
        for (const ExtrusionLine &extrusion_line : perimeter) {
            if (extrusion_line.empty())
                continue; // This shouldn't ever happen.

            const BoundingBox bbox    = get_extents(extrusion_line);
            // Be aware that Arachne produces contours with clockwise orientation and holes with counterclockwise orientation.
            const double      area    = std::abs(extrusion_line.area());
            const Polygon     polygon = extrusion_line.is_closed ? to_polygon(extrusion_line) : Polygon{};

            sorted_perimeter_extrusions.emplace_back(extrusion_line, area, polygon, bbox);
        }
    }

    // Open extrusions have an area equal to zero, so sorting based on the area ensures that open extrusions will always be before closed ones.
    std::sort(sorted_perimeter_extrusions.begin(), sorted_perimeter_extrusions.end(),
              [](const PerimeterExtrusion &l, const PerimeterExtrusion &r) { return l.area < r.area; });

    return sorted_perimeter_extrusions;
}

// Analyze perimeter structure by depth instead of simple array positions
static PerimeterDepthLayer analyze_perimeter_structure(
    const std::vector<const PerimeterExtrusion*>& extrusions)
{
    PerimeterDepthLayer layers;

    for (const auto* extrusion : extrusions) {
        // Categorize by actual depth, not array position
        if (extrusion->is_external_perimeter()) {
            layers.externals.push_back(extrusion);
        } else if (extrusion->is_first_internal_perimeter()) {
            layers.first_internals.push_back(extrusion);
        } else if (extrusion->is_second_internal_perimeter()) {
            layers.second_internals.push_back(extrusion);
        } else {
            layers.other_internals.push_back(extrusion);
        }
    }

    return layers;
}

// Order multiple loops at the same depth by proximity to minimize travel
static std::vector<const PerimeterExtrusion*> order_by_proximity(
    const std::vector<const PerimeterExtrusion*>& extrusions,
    const Point& start_position)
{
    if (extrusions.size() <= 1) return extrusions;

    std::vector<const PerimeterExtrusion*> ordered;
    std::vector<bool> used(extrusions.size(), false);
    Point current_pos = start_position;

    while (ordered.size() < extrusions.size()) {
        double min_dist = std::numeric_limits<double>::max();
        size_t nearest_idx = 0;

        for (size_t i = 0; i < extrusions.size(); ++i) {
            if (used[i]) continue;

            const Point& extrusion_start = extrusions[i]->extrusion.junctions.front().p;
            double dist = (current_pos - extrusion_start).cast<double>().squaredNorm();

            if (dist < min_dist) {
                min_dist = dist;
                nearest_idx = i;
            }
        }

        ordered.push_back(extrusions[nearest_idx]);
        used[nearest_idx] = true;
        current_pos = get_end_position(extrusions[nearest_idx]->extrusion);
    }

    return ordered;
}

// Analyze relationship between multiple groups
static MultiGroupPattern analyze_group_relationships(
    const std::vector<GroupedPerimeterExtrusions>& groups)
{
    if (groups.size() <= 1) return MultiGroupPattern::INDEPENDENT;

    // Check if groups are concentric (typical for tubes/holes)
    bool has_contours = false;
    bool has_holes = false;

    for (const auto& group : groups) {
        if (group.external_perimeter_extrusion->is_contour()) {
            has_contours = true;
        } else {
            has_holes = true;
        }
    }

    if (has_contours && has_holes) {
        // Check perimeter counts to determine if it's a thin structure
        int avg_perimeters = 0;
        for (const auto& group : groups) {
            avg_perimeters += group.extrusions.size();
        }
        avg_perimeters /= groups.size();

        if (avg_perimeters <= 2) {
            // Thin concentric structure (like thin tube)
            return MultiGroupPattern::CONCENTRIC;
        }
    }

    // TODO: Detect ADJACENT patterns by checking bbox overlaps

    return MultiGroupPattern::INDEPENDENT;
}

// Apply injection molding strategy based on perimeter structure
static void apply_injection_molding_order(
    GroupedPerimeterExtrusions& group,
    bool enable_injection_molding,
    bool is_overhang_region = false)
{
    if (!enable_injection_molding) {
        return; // Keep original order
    }

    // Analyze structure by depth
    PerimeterDepthLayer layers = analyze_perimeter_structure(group.extrusions);

    // Determine strategy based on what we have
    if (layers.has_groove_structure()) {
        // ===== GROOVE INJECTION STRATEGY (3+ perimeters) =====
        // Goal: Create groove with External + Second Internal, inject First Internal
        // Before reverse: [First_internals, Externals, Second_internals, Others...]
        // After reverse: [...Others, Second_internals, Externals, First_internals(HOT)]

        std::vector<const PerimeterExtrusion*> reordered;
        Point current_pos = Point::Zero();

        // 1. All first internals (will be last after reverse, printed HOT)
        auto ordered_first = order_by_proximity(layers.first_internals, current_pos);
        reordered.insert(reordered.end(), ordered_first.begin(), ordered_first.end());
        if (!ordered_first.empty()) {
            current_pos = get_end_position(ordered_first.back()->extrusion);
        }

        // 2. All externals (will form outer wall of groove)
        auto ordered_externals = order_by_proximity(layers.externals, current_pos);
        reordered.insert(reordered.end(), ordered_externals.begin(), ordered_externals.end());
        if (!ordered_externals.empty()) {
            current_pos = get_end_position(ordered_externals.back()->extrusion);
        }

        // 3. All second internals (will form inner wall of groove)
        auto ordered_seconds = order_by_proximity(layers.second_internals, current_pos);
        reordered.insert(reordered.end(), ordered_seconds.begin(), ordered_seconds.end());
        if (!ordered_seconds.empty()) {
            current_pos = get_end_position(ordered_seconds.back()->extrusion);
        }

        // 4. All other internals (structural support)
        auto ordered_others = order_by_proximity(layers.other_internals, current_pos);
        reordered.insert(reordered.end(), ordered_others.begin(), ordered_others.end());

        group.extrusions = reordered;

    } else if (layers.is_simple_two_perimeter()) {
        // ===== TWO PERIMETER STRATEGY (no groove possible) =====
        // Only swap for overhangs, otherwise maintain normal order

        if (is_overhang_region && group.extrusions.size() >= 2) {
            // Simple swap for better overhang support
            std::swap(group.extrusions[0], group.extrusions[1]);
        }
        // Otherwise keep original order - no benefit without groove

    } else if (layers.externals.size() > 1 || layers.first_internals.size() > 1) {
        // ===== MULTIPLE LOOPS AT SAME DEPTH =====
        // Keep loops at same depth together, order by proximity

        std::vector<const PerimeterExtrusion*> reordered;
        Point current_pos = Point::Zero();

        // When we have multiple loops at the same depth, keep them grouped
        // This ensures consistent treatment (same temperature, etc.)

        // Order: First internals → Externals → Second internals → Others
        // (This will be reversed later if !external_perimeters_first)

        auto ordered_first = order_by_proximity(layers.first_internals, current_pos);
        reordered.insert(reordered.end(), ordered_first.begin(), ordered_first.end());
        if (!ordered_first.empty()) {
            current_pos = get_end_position(ordered_first.back()->extrusion);
        }

        auto ordered_externals = order_by_proximity(layers.externals, current_pos);
        reordered.insert(reordered.end(), ordered_externals.begin(), ordered_externals.end());
        if (!ordered_externals.empty()) {
            current_pos = get_end_position(ordered_externals.back()->extrusion);
        }

        auto ordered_seconds = order_by_proximity(layers.second_internals, current_pos);
        reordered.insert(reordered.end(), ordered_seconds.begin(), ordered_seconds.end());
        if (!ordered_seconds.empty()) {
            current_pos = get_end_position(ordered_seconds.back()->extrusion);
        }

        auto ordered_others = order_by_proximity(layers.other_internals, current_pos);
        reordered.insert(reordered.end(), ordered_others.begin(), ordered_others.end());

        group.extrusions = reordered;
    }
    // Single perimeter - nothing to reorder
}

// Handle special multi-group patterns
static void handle_multi_group_patterns(
    std::vector<GroupedPerimeterExtrusions>& groups,
    MultiGroupPattern pattern,
    bool enable_injection_molding)
{
    if (pattern == MultiGroupPattern::CONCENTRIC && groups.size() > 1) {
        // For concentric patterns with limited perimeters (like thin tubes)
        // Reorder groups to optimize the shared perimeter scenario

        // Count total perimeters across all groups
        int total_perimeters = 0;
        for (const auto& group : groups) {
            total_perimeters += group.extrusions.size();
        }

        if (total_perimeters <= 3) {
            // Thin concentric structure: Optimize group order
            // Put contours before holes for better printing
            std::sort(groups.begin(), groups.end(),
                      [](const GroupedPerimeterExtrusions& a,
                         const GroupedPerimeterExtrusions& b) {
                          // Contours (true) before holes (false)
                          return a.external_perimeter_extrusion->is_contour() >
                          b.external_perimeter_extrusion->is_contour();
                         });
        }
    }
    // Other patterns can be handled here as needed
}


// Functions fill adjacent_perimeter_extrusions field for every PerimeterExtrusion by pointers to PerimeterExtrusions that contain or are inside this PerimeterExtrusion.
static void construct_perimeter_extrusions_adjacency_graph(PerimeterExtrusions &sorted_perimeter_extrusions) {
    // Construct a graph (defined using adjacent_perimeter_extrusions field) where two PerimeterExtrusion are adjacent when one is inside the other.
    std::vector<bool> root_candidates(sorted_perimeter_extrusions.size(), false);
    for (PerimeterExtrusion &perimeter_extrusion : sorted_perimeter_extrusions) {
        const size_t perimeter_extrusion_idx = &perimeter_extrusion - sorted_perimeter_extrusions.data();

        if (!perimeter_extrusion.is_closed()) {
            root_candidates[perimeter_extrusion_idx] = true;
            continue;
        }

        for (PerimeterExtrusion &root_candidate : sorted_perimeter_extrusions) {
            const size_t root_candidate_idx = &root_candidate - sorted_perimeter_extrusions.data();

            if (!root_candidates[root_candidate_idx])
                continue;

            if (perimeter_extrusion.bbox.contains(root_candidate.bbox) && perimeter_extrusion.polygon.contains(root_candidate.extrusion.junctions.front().p)) {
                perimeter_extrusion.adjacent_perimeter_extrusions.emplace_back(&root_candidate);
                root_candidate.adjacent_perimeter_extrusions.emplace_back(&perimeter_extrusion);
                root_candidates[root_candidate_idx] = false;
            }
        }

        root_candidates[perimeter_extrusion_idx] = true;
    }
}

// Perform the depth-first search to assign the nearest external perimeter for every PerimeterExtrusion.
// When some PerimeterExtrusion is achievable from more than one external perimeter, then we choose the
// one that comes from a contour.
static void assign_nearest_external_perimeter(PerimeterExtrusions &sorted_perimeter_extrusions) {
    std::stack<PerimeterExtrusion *> stack;
    for (PerimeterExtrusion &perimeter_extrusion : sorted_perimeter_extrusions) {
        if (perimeter_extrusion.is_external_perimeter()) {
            perimeter_extrusion.depth                      = 0;
            perimeter_extrusion.nearest_external_perimeter = &perimeter_extrusion;
            stack.push(&perimeter_extrusion);
        }
    }

    while (!stack.empty()) {
        PerimeterExtrusion *current_extrusion = stack.top();
        stack.pop();

        for (PerimeterExtrusion *adjacent_extrusion : current_extrusion->adjacent_perimeter_extrusions) {
            const size_t adjacent_extrusion_depth = current_extrusion->depth + 1;
            // Update depth when the new depth is smaller or when we can achieve the same depth from a contour.
            // This will ensure that the internal perimeter will be extruded before the outer external perimeter
            // when there are two external perimeters and one internal.
            if (adjacent_extrusion_depth < adjacent_extrusion->depth) {
                adjacent_extrusion->nearest_external_perimeter = current_extrusion->nearest_external_perimeter;
                adjacent_extrusion->depth                      = adjacent_extrusion_depth;
                stack.push(adjacent_extrusion);
            } else if (adjacent_extrusion_depth == adjacent_extrusion->depth && !adjacent_extrusion->nearest_external_perimeter->is_contour() && current_extrusion->is_contour()) {
                adjacent_extrusion->nearest_external_perimeter = current_extrusion->nearest_external_perimeter;
                stack.push(adjacent_extrusion);
            }
        }
    }
}

inline Point get_end_position(const ExtrusionLine &extrusion) {
    if (extrusion.is_closed)
        return extrusion.junctions[0].p; // We ended where we started.
    else
        return extrusion.junctions.back().p; // Pick the other end from where we started.
}

// Returns ordered extrusions.
static std::vector<const PerimeterExtrusion *> ordered_perimeter_extrusions_to_minimize_distances(Point current_position, std::vector<const PerimeterExtrusion *> extrusions) {
    // Ensure that open extrusions will be placed before the closed one.
    std::sort(extrusions.begin(), extrusions.end(),
              [](const PerimeterExtrusion *l, const PerimeterExtrusion *r) -> bool { return l->is_closed() < r->is_closed(); });

    std::vector<const PerimeterExtrusion *> ordered_extrusions;
    std::vector<bool> already_selected(extrusions.size(), false);
    while (ordered_extrusions.size() < extrusions.size()) {
        double nearest_distance_sqr  = std::numeric_limits<double>::max();
        size_t nearest_extrusion_idx = 0;
        bool   is_nearest_closed     = false;

        for (size_t extrusion_idx = 0; extrusion_idx < extrusions.size(); ++extrusion_idx) {
            if (already_selected[extrusion_idx])
                continue;

            const ExtrusionLine &extrusion_line           = extrusions[extrusion_idx]->extrusion;
            const Point         &extrusion_start_position = extrusion_line.junctions.front().p;
            const double         distance_sqr             = (current_position - extrusion_start_position).cast<double>().squaredNorm();
            if (distance_sqr < nearest_distance_sqr) {
                if (extrusion_line.is_closed || (!extrusion_line.is_closed && nearest_distance_sqr == std::numeric_limits<double>::max()) || (!extrusion_line.is_closed && !is_nearest_closed)) {
                    nearest_extrusion_idx = extrusion_idx;
                    nearest_distance_sqr  = distance_sqr;
                    is_nearest_closed     = extrusion_line.is_closed;
                }
            }
        }

        already_selected[nearest_extrusion_idx]     = true;
        const PerimeterExtrusion *nearest_extrusion = extrusions[nearest_extrusion_idx];
        current_position                            = get_end_position(nearest_extrusion->extrusion);
        ordered_extrusions.emplace_back(nearest_extrusion);
    }

    return ordered_extrusions;
}



// Returns vector of indexes that represent the order of grouped extrusions in grouped_extrusions.
static std::vector<size_t> order_of_grouped_perimeter_extrusions_to_minimize_distances(const std::vector<GroupedPerimeterExtrusions> &grouped_extrusions, Point current_position) {
    std::vector<size_t> grouped_extrusions_sorted_indices(grouped_extrusions.size());
    std::iota(grouped_extrusions_sorted_indices.begin(), grouped_extrusions_sorted_indices.end(), 0);

    // Ensure that holes will be placed before contour and open extrusions before the closed one.
    std::sort(grouped_extrusions_sorted_indices.begin(), grouped_extrusions_sorted_indices.end(), [&grouped_extrusions = std::as_const(grouped_extrusions)](const size_t l_idx, const size_t r_idx) -> bool {
        const GroupedPerimeterExtrusions &l = grouped_extrusions[l_idx];
        const GroupedPerimeterExtrusions &r = grouped_extrusions[r_idx];
        return (l.external_perimeter_extrusion->is_contour() <  r.external_perimeter_extrusion->is_contour()) ||
               (l.external_perimeter_extrusion->is_contour() == r.external_perimeter_extrusion->is_contour()  && l.external_perimeter_extrusion->is_closed() < r.external_perimeter_extrusion->is_closed());
    });

    const size_t holes_cnt = std::count_if(grouped_extrusions.begin(), grouped_extrusions.end(), [](const GroupedPerimeterExtrusions &grouped_extrusions) {
        return !grouped_extrusions.external_perimeter_extrusion->is_contour();
    });

    std::vector<size_t> grouped_extrusions_order;
    std::vector<bool>   already_selected(grouped_extrusions.size(), false);
    while (grouped_extrusions_order.size() < grouped_extrusions.size()) {
        double nearest_distance_sqr           = std::numeric_limits<double>::max();
        size_t nearest_grouped_extrusions_idx = 0;
        bool   is_nearest_closed              = false;

        // First we order all holes and then we start ordering contours.
        const size_t grouped_extrusions_sorted_indices_end = (grouped_extrusions_order.size() < holes_cnt) ? holes_cnt : grouped_extrusions_sorted_indices.size();
        for (size_t grouped_extrusions_sorted_idx = 0; grouped_extrusions_sorted_idx < grouped_extrusions_sorted_indices_end; ++grouped_extrusions_sorted_idx) {
            const size_t grouped_extrusion_idx = grouped_extrusions_sorted_indices[grouped_extrusions_sorted_idx];
            if (already_selected[grouped_extrusion_idx])
                continue;

            const ExtrusionLine &external_perimeter_extrusion_line = grouped_extrusions[grouped_extrusion_idx].external_perimeter_extrusion->extrusion;
            const Point         &extrusion_start_position          = external_perimeter_extrusion_line.junctions.front().p;
            const double         distance_sqr                      = (current_position - extrusion_start_position).cast<double>().squaredNorm();
            if (distance_sqr < nearest_distance_sqr) {
                if (external_perimeter_extrusion_line.is_closed || (!external_perimeter_extrusion_line.is_closed && nearest_distance_sqr == std::numeric_limits<double>::max()) || (!external_perimeter_extrusion_line.is_closed && !is_nearest_closed)) {
                    nearest_grouped_extrusions_idx = grouped_extrusion_idx;
                    nearest_distance_sqr           = distance_sqr;
                    is_nearest_closed              = external_perimeter_extrusion_line.is_closed;
                }
            }
        }

        grouped_extrusions_order.emplace_back(nearest_grouped_extrusions_idx);
        already_selected[nearest_grouped_extrusions_idx]             = true;

        const GroupedPerimeterExtrusions &nearest_grouped_extrusions = grouped_extrusions[nearest_grouped_extrusions_idx];
        const ExtrusionLine              &last_extrusion_line        = nearest_grouped_extrusions.extrusions.back()->extrusion;
        current_position                                             = get_end_position(last_extrusion_line);
    }

    return grouped_extrusions_order;
}

static PerimeterExtrusions extract_ordered_perimeter_extrusions(const PerimeterExtrusions &sorted_perimeter_extrusions, const bool external_perimeters_first, const bool enable_injection_molding, const bool reverse_internal_perimeters, const int reverse_internal_perimeters_at) {
    // Extrusions are ordered inside each group.
    std::vector<GroupedPerimeterExtrusions> grouped_extrusions;

    std::stack<const PerimeterExtrusion *> stack;
    std::vector<bool>                      visited(sorted_perimeter_extrusions.size(), false);
    for (const PerimeterExtrusion &perimeter_extrusion : sorted_perimeter_extrusions) {
        if (!perimeter_extrusion.is_external_perimeter())
            continue;

        stack.push(&perimeter_extrusion);
        visited.assign(sorted_perimeter_extrusions.size(), false);

        grouped_extrusions.emplace_back(&perimeter_extrusion);
        while (!stack.empty()) {
            const PerimeterExtrusion *current_extrusion     = stack.top();
            const size_t              current_extrusion_idx = current_extrusion - sorted_perimeter_extrusions.data();

            stack.pop();
            visited[current_extrusion_idx] = true;

            if (current_extrusion->nearest_external_perimeter == &perimeter_extrusion) {
                grouped_extrusions.back().extrusions.emplace_back(current_extrusion);
            }

            std::vector<const PerimeterExtrusion *> available_candidates;
            for (const PerimeterExtrusion *adjacent_extrusion : current_extrusion->adjacent_perimeter_extrusions) {
                const size_t adjacent_extrusion_idx = adjacent_extrusion - sorted_perimeter_extrusions.data();
                if (!visited[adjacent_extrusion_idx] && !adjacent_extrusion->is_external_perimeter() && adjacent_extrusion->nearest_external_perimeter == &perimeter_extrusion) {
                    available_candidates.emplace_back(adjacent_extrusion);
                }
            }

            if (available_candidates.size() == 1) {
                stack.push(available_candidates.front());
            } else if (available_candidates.size() > 1) {
                // When there is more than one available candidate, then order candidates to minimize distances between
                // candidates and also to minimize the distance from the current_position.
                std::vector<const PerimeterExtrusion *> adjacent_extrusions = ordered_perimeter_extrusions_to_minimize_distances(Point::Zero(), available_candidates);
                for (auto extrusion_it = adjacent_extrusions.rbegin(); extrusion_it != adjacent_extrusions.rend(); ++extrusion_it) {
                    stack.push(*extrusion_it);
                }
            }
        } 
        if (enable_injection_molding) {
            // Detect if this is an overhang region
            bool is_overhang = false;
            for (const auto* extrusion : grouped_extrusions.back().extrusions) {
                // Check for overhang role
                if (extrusion->extrusion.inset_idx == 0) {  // External
                    // Could check for Bridge modifier or other overhang indicators
                    // Simplified check here
                }
            }

            // Apply advanced injection molding order
            apply_injection_molding_order(grouped_extrusions.back(), true, is_overhang);
        }
        if (reverse_internal_perimeters){
            if ( grouped_extrusions.back().extrusions.size() > (unsigned long) (unsigned int) reverse_internal_perimeters_at ) {
                std::reverse(grouped_extrusions.back().extrusions.begin()+reverse_internal_perimeters_at, grouped_extrusions.back().extrusions.end());
                }
        }
        if ( !external_perimeters_first )
            std::reverse(grouped_extrusions.back().extrusions.begin(), grouped_extrusions.back().extrusions.end());
    }

    // ===== Multi-group pattern handling ONLY if injection molding enabled =====
    if (enable_injection_molding) {
        MultiGroupPattern pattern = analyze_group_relationships(grouped_extrusions);
        handle_multi_group_patterns(grouped_extrusions, pattern, true);
    }


    const std::vector<size_t> grouped_extrusion_order = order_of_grouped_perimeter_extrusions_to_minimize_distances(grouped_extrusions, Point::Zero());

    PerimeterExtrusions ordered_extrusions;
    for (size_t order_idx : grouped_extrusion_order) {
        for (const PerimeterExtrusion *perimeter_extrusion : grouped_extrusions[order_idx].extrusions)
            ordered_extrusions.emplace_back(*perimeter_extrusion);
    }

    return ordered_extrusions;
}

// FIXME: From the point of better patch planning, it should be better to do ordering when we have generated all extrusions (for now, when G-Code is exported).
// FIXME: It would be better to extract the adjacency graph of extrusions from the SkeletalTrapezoidation graph.
PerimeterExtrusions ordered_perimeter_extrusions(const Perimeters &perimeters, const bool external_perimeters_first, const bool enable_injection_molding, const bool reverse_internal_perimeters, const int reverse_internal_perimeters_at) {
    PerimeterExtrusions sorted_perimeter_extrusions = get_sorted_perimeter_extrusions_by_area(perimeters);
    construct_perimeter_extrusions_adjacency_graph(sorted_perimeter_extrusions);
    assign_nearest_external_perimeter(sorted_perimeter_extrusions);
    return extract_ordered_perimeter_extrusions(sorted_perimeter_extrusions, external_perimeters_first, enable_injection_molding, reverse_internal_perimeters, reverse_internal_perimeters_at);
}

} // namespace Slic3r::Arachne::PerimeterOrder

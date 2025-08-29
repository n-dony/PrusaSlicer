#include <stack>
#include <algorithm>
#include <cmath>

#include "PerimeterOrder.hpp"
#include "libslic3r/Arachne/utils/ExtrusionJunction.hpp"
#include "libslic3r/Point.hpp"

namespace Slic3r::Arachne::PerimeterOrder {

using namespace Arachne;

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

struct GroupedPerimeterExtrusions
{
    GroupedPerimeterExtrusions() = delete;
    explicit GroupedPerimeterExtrusions(const PerimeterExtrusion *external_perimeter_extrusion)
        : external_perimeter_extrusion(external_perimeter_extrusion) {}

    std::vector<const PerimeterExtrusion *> extrusions;
    const PerimeterExtrusion               *external_perimeter_extrusion  = nullptr;
};

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

// In PerimeterOrder.cpp
static PerimeterExtrusions extract_ordered_perimeter_extrusions(
    const PerimeterExtrusions &sorted_perimeter_extrusions, 
    const bool external_perimeters_first, 
    const bool swap_first_int_w_ext_perimeter, 
    const bool reverse_internal_perimeters, 
    const int reverse_internal_perimeters_at) {
    
    // ===== PHASE 1: BUILD GROUPS (unchanged) =====
    std::vector<GroupedPerimeterExtrusions> grouped_extrusions;
    std::vector<bool> visited(sorted_perimeter_extrusions.size(), false);
    std::deque<const PerimeterExtrusion *> stack;

    for (size_t seed_idx = 0; seed_idx < sorted_perimeter_extrusions.size(); seed_idx++) {
        if (!visited[seed_idx]) {
            const PerimeterExtrusion *seed_perimeter_extrusion = &sorted_perimeter_extrusions[seed_idx];
            visited[seed_idx] = true;
            stack.push_back(seed_perimeter_extrusion);

            grouped_extrusions.push_back(GroupedPerimeterExtrusions(seed_perimeter_extrusion));

            while (!stack.empty()) {
                const PerimeterExtrusion *perimeter_extrusion = stack.front();
                stack.pop_front();

                grouped_extrusions.back().extrusions.push_back(perimeter_extrusion);

                std::vector<const PerimeterExtrusion *> external_candidates;
                std::vector<const PerimeterExtrusion *> other_candidates;

                for (const PerimeterExtrusion *adjacent_extrusion : perimeter_extrusion->adjacent_perimeter_extrusions) {
                    const size_t adjacent_idx = adjacent_extrusion - sorted_perimeter_extrusions.data();
                    if (!visited[adjacent_idx]) {
                        visited[adjacent_idx] = true;
                        if (adjacent_extrusion->is_external_perimeter())
                            external_candidates.push_back(adjacent_extrusion);
                        else
                            other_candidates.push_back(adjacent_extrusion);
                    }
                }

                std::vector<const PerimeterExtrusion *> available_candidates;
                append(available_candidates, external_candidates);
                append(available_candidates, other_candidates);
                std::vector<const PerimeterExtrusion *> adjacent_extrusions = 
                    ordered_perimeter_extrusions_to_minimize_distances(Point::Zero(), available_candidates);
                for (auto extrusion_it = adjacent_extrusions.rbegin(); extrusion_it != adjacent_extrusions.rend(); ++extrusion_it) {
                    stack.push_back(*extrusion_it);
                }
            }
        }
    }
    
    // ===== PHASE 2: DEBUG HELPERS =====
    #ifdef DEBUG
    auto log_perimeter_order = [](const std::vector<const PerimeterExtrusion*>& perims, const std::string& phase) {
        printf("%s: ", phase.c_str());
        for (const auto* p : perims) {
            if (p->is_external_perimeter()) printf("Ext(d%zu) ", p->depth);
            else if (p->is_first_internal_perimeter()) printf("Int1(d%zu) ", p->depth);
            else if (p->is_second_internal_perimeter()) printf("Int2(d%zu) ", p->depth);
            else printf("Int%zu(d%zu) ", p->depth, p->depth);
        }
        printf("\n");
    };
    
    printf("\n=== Perimeter Ordering Debug ===\n");
    printf("Total groups: %zu\n", grouped_extrusions.size());
    printf("Options: external_first=%d, swap=%d, reverse=%d(at %d)\n",
           external_perimeters_first, swap_first_int_w_ext_perimeter, 
           reverse_internal_perimeters, reverse_internal_perimeters_at);
    #endif
    
    // ===== PHASE 3: PROCESS EACH GROUP =====
    for (size_t group_idx = 0; group_idx < grouped_extrusions.size(); group_idx++) {
        auto& group = grouped_extrusions[group_idx];
        
        #ifdef DEBUG
        printf("\n--- Group %zu (%zu perimeters) ---\n", group_idx, group.extrusions.size());
        log_perimeter_order(group.extrusions, "Original");
        #endif
        
        // Step 1: Sort by depth (deepest first for inside-out default)
        // We HAVE this information - use it!
        std::stable_sort(group.extrusions.begin(), group.extrusions.end(),
                        [](const PerimeterExtrusion* a, const PerimeterExtrusion* b) {
                            return a->depth > b->depth;  // Higher depth = deeper internal
                        });
        
        #ifdef DEBUG
        log_perimeter_order(group.extrusions, "After depth sort");
        #endif
        
        // Step 2: Apply reverse_internal_perimeters
        // reverse_internal_perimeters_at counts from OUTSIDE:
        // 1 = reverse ALL internals (depth >= 1)
        // 2 = reverse from second internal (depth >= 2)
        // 3 = reverse from third internal (depth >= 3)
        if (reverse_internal_perimeters && reverse_internal_perimeters_at > 0) {
            // Find where to start reversing
            auto reverse_start = std::stable_partition(
                group.extrusions.begin(), group.extrusions.end(),
                [reverse_internal_perimeters_at](const PerimeterExtrusion* p) {
                    // Keep perimeters that should NOT be reversed
                    return p->depth < reverse_internal_perimeters_at;
                });
            
            // Reverse the selected range
            if (reverse_start != group.extrusions.end()) {
                std::reverse(reverse_start, group.extrusions.end());
                
                #ifdef DEBUG
                printf("Reversed %zu perimeters from depth %d\n", 
                       std::distance(reverse_start, group.extrusions.end()),
                       reverse_internal_perimeters_at);
                log_perimeter_order(group.extrusions, "After reverse");
                #endif
            }
        }
        
        // Step 3: Apply swap_first_int_w_ext_perimeter (GROOVE INJECTION)
        // Move ALL first_internal (depth=1) to the END
        if (swap_first_int_w_ext_perimeter) {
            // Partition: everything except first_internal goes first
            auto first_internal_start = std::stable_partition(
                group.extrusions.begin(), group.extrusions.end(),
                [](const PerimeterExtrusion* p) {
                    return p->depth != 1;  // Keep if NOT first_internal
                });
            
            // Now first_internal perimeters are at the end
            
            #ifdef DEBUG
            if (first_internal_start != group.extrusions.end()) {
                printf("Moved %zu first_internal(s) to end for groove injection\n",
                       std::distance(first_internal_start, group.extrusions.end()));
            }
            log_perimeter_order(group.extrusions, "After swap (groove injection)");
            #endif
        }
        
        // Step 4: Apply external_perimeters_first
        // Note: external_perimeters_first is actually a reversal flag
        // false = inside-out (default, what we have now)
        // true = outside-in (need to reverse)
        if (external_perimeters_first) {
            std::reverse(group.extrusions.begin(), group.extrusions.end());
            #ifdef DEBUG
            printf("Reversed for external_perimeters_first (outside-in)\n");
            log_perimeter_order(group.extrusions, "Final");
            #endif
        } else {
            #ifdef DEBUG
            log_perimeter_order(group.extrusions, "Final (inside-out)");
            #endif
        }
    }
    
    // ===== PHASE 4: INTER-GROUP HANDLING =====
    // For complex geometries with multiple groups
    if (grouped_extrusions.size() > 1 && swap_first_int_w_ext_perimeter) {
        #ifdef DEBUG
        printf("\n=== Inter-group processing ===\n");
        #endif
        
        // Collect all first_internals from all groups
        std::vector<const PerimeterExtrusion*> all_first_internals;
        
        for (auto& group : grouped_extrusions) {
            auto new_end = std::remove_if(
                group.extrusions.begin(), group.extrusions.end(),
                [&all_first_internals](const PerimeterExtrusion* p) {
                    if (p->depth == 1) {  // first_internal
                        all_first_internals.push_back(p);
                        return true;
                    }
                    return false;
                });
            group.extrusions.erase(new_end, group.extrusions.end());
        }
        
        // Add all first_internals to the last non-empty group
        if (!all_first_internals.empty()) {
            // Find last non-empty group
            auto last_non_empty = std::find_if(
                grouped_extrusions.rbegin(), grouped_extrusions.rend(),
                [](const GroupedPerimeterExtrusions& g) { 
                    return !g.extrusions.empty(); 
                });
            
            if (last_non_empty != grouped_extrusions.rend()) {
                last_non_empty->extrusions.insert(
                    last_non_empty->extrusions.end(),
                    all_first_internals.begin(), 
                    all_first_internals.end()
                );
                
                #ifdef DEBUG
                printf("Moved %zu first_internals to last group for injection\n", 
                       all_first_internals.size());
                #endif
            }
        }
        
        // Remove empty groups
        grouped_extrusions.erase(
            std::remove_if(grouped_extrusions.begin(), grouped_extrusions.end(),
                          [](const GroupedPerimeterExtrusions& g) { 
                              return g.extrusions.empty(); 
                          }),
            grouped_extrusions.end()
        );
    }
    
    // ===== PHASE 5: BUILD FINAL OUTPUT =====
    const std::vector<size_t> grouped_extrusion_order = 
        order_of_grouped_perimeter_extrusions_to_minimize_distances(grouped_extrusions, Point::Zero());
    
    PerimeterExtrusions ordered_extrusions;
    ordered_extrusions.reserve(sorted_perimeter_extrusions.size());
    
    for (size_t order_idx : grouped_extrusion_order) {
        for (const PerimeterExtrusion *perimeter_extrusion : grouped_extrusions[order_idx].extrusions) {
            ordered_extrusions.emplace_back(*perimeter_extrusion);
        }
    }
    
    #ifdef DEBUG
    printf("\n=== Final result: %zu perimeters total ===\n", ordered_extrusions.size());
    #endif
    
    return ordered_extrusions;
}

// FIXME: From the point of better patch planning, it should be better to do ordering when we have generated all extrusions (for now, when G-Code is exported).
// FIXME: It would be better to extract the adjacency graph of extrusions from the SkeletalTrapezoidation graph.
PerimeterExtrusions ordered_perimeter_extrusions(const Perimeters &perimeters, const bool external_perimeters_first, const bool swap_first_int_w_ext_perimeter, const bool reverse_internal_perimeters, const int reverse_internal_perimeters_at) {
    PerimeterExtrusions sorted_perimeter_extrusions = get_sorted_perimeter_extrusions_by_area(perimeters);
    construct_perimeter_extrusions_adjacency_graph(sorted_perimeter_extrusions);
    assign_nearest_external_perimeter(sorted_perimeter_extrusions);
    return extract_ordered_perimeter_extrusions(sorted_perimeter_extrusions, external_perimeters_first, swap_first_int_w_ext_perimeter, reverse_internal_perimeters, reverse_internal_perimeters_at);
}

} // namespace Slic3r::Arachne::PerimeterOrder

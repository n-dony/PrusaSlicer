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
// In PerimeterOrder.hpp - FUTURE ENHANCEMENT
struct PerimeterExtrusion
{
    // Existing fields
    Arachne::ExtrusionLine extrusion;
    double area;
    Polygon polygon;
    BoundingBox bbox;
    size_t depth;  // WE ALREADY HAVE THIS!
    std::vector<PerimeterExtrusion *> adjacent_perimeter_extrusions;
    PerimeterExtrusion *nearest_external_perimeter;

    PerimeterExtrusion(const ExtrusionLine& line, double a, const Polygon& poly, const BoundingBox& box)
        : extrusion(line), area(a), polygon(poly), bbox(box) {}
    
        // Default constructor
    PerimeterExtrusion() = default;


    // Existing methods
    bool is_contour() const { return extrusion.is_contour(); }
    bool is_closed() const { return extrusion.is_closed; }
    bool is_external_perimeter() const { return extrusion.is_external_perimeter(); }
    bool is_first_internal_perimeter() const { return extrusion.is_first_internal_perimeter(); }
    bool is_second_internal_perimeter() const { return extrusion.is_second_internal_perimeter(); }
    
    // NEW: Extended metadata (FUTURE)
    struct ExtendedMetadata {
        // Wall/Island grouping
        size_t wall_id;              // Which wall this belongs to
        size_t region_id;            // Which region in layer
        bool is_hole;                // Hole or contour
        
        // Thermal properties
        float cooling_time_needed;   // Before next perimeter
        float optimal_temperature;   // Calculated optimal
        bool is_overhang;           
        float overhang_percentage;   // 0-100%
        
        // Flow properties
        float line_width;           
        float flow_rate;            
        float layer_height;         
        
        // Sequencing hints
        bool can_be_injected;       // Can be printed last
        bool needs_support;         
        int suggested_order;        
        
        // Quality metrics
        bool is_visible;            
        bool is_structural;         
        bool is_bridging;           
    } metadata;
    
    // Helper method
    int perimeter_number() const {
        // Returns which internal perimeter this is
        // 0 = external, 1 = first_internal, 2 = second_internal, etc.
        return depth;
    }

    

};
/*
struct WallGroup {
    std::vector<PerimeterExtrusion*> perimeters;
    size_t wall_id;
    size_t region_id;
    
    // Apply ordering within THIS WALL only
    void optimize_order(const PrintRegionConfig& config) {
        // Sort by depth
        std::sort(perimeters.begin(), perimeters.end(),
                 [](const PerimeterExtrusion* a, const PerimeterExtrusion* b) {
                     return a->depth > b->depth;
                 });
        
        // Apply operations to THIS WALL's perimeters
        if (config.reverse_internal_perimeters) {
            apply_reverse_internal(config.reverse_internal_perimeters_at);
        }
        
        if (config.swap_first_int_w_ext_perimeter) {
            apply_groove_injection();
        }
        
        if (config.external_perimeters_first) {
            std::reverse(perimeters.begin(), perimeters.end());
        }
    }
    
private:
    void apply_groove_injection() {
        // Move depth=1 perimeters to end
        std::stable_partition(perimeters.begin(), perimeters.end(),
                             [](const PerimeterExtrusion* p) { 
                                 return p->depth != 1; 
                             });
    }
    
    void apply_reverse_internal(int at_depth) {
        auto start = std::find_if(perimeters.begin(), perimeters.end(),
                                 [at_depth](const PerimeterExtrusion* p) {
                                     return p->depth >= at_depth && p->depth > 0;
                                 });
        if (start != perimeters.end()) {
            auto end = std::find_if(start, perimeters.end(),
                                   [](const PerimeterExtrusion* p) {
                                       return p->depth == 0;
                                   });
            std::reverse(start, end);
        }
    }
};
*/
using PerimeterExtrusions = std::vector<PerimeterExtrusion>;

PerimeterExtrusions ordered_perimeter_extrusions(const Perimeters &perimeters, bool external_perimeters_first,  bool swap_first_int_w_ext_perimeter,  bool reverse_internal_perimeters, int reverse_internal_perimeters_at);

} // namespace Slic3r::Arachne::PerimeterOrder

#endif // slic3r_GCode_Travels_hpp_

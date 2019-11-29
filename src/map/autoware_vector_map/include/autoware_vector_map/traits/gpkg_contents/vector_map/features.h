#pragma once

#include <autoware_vector_map/traits/gpkg_contents/gpkg_contents.h>

#define AUTOWARE_VECTOR_MAP_REGISTER_GPKG_FEATURE(FEATURE, TABLE_NAME) \
  AUTOWARE_VECTOR_MAP_REGISTER_GPKG_CONTENT(FEATURE, TABLE_NAME)

namespace autoware_vector_map {
namespace traits {

template <class T>
struct gpkg_feature {};

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_FEATURE(IntersectionArea, intersection_areas)

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_FEATURE(LaneSection, lane_sections)

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_FEATURE(Lane, lanes)
AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(Lane, 0, lane_section_id, data::invalid_id)
AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(Lane, 1, width, 0.0)
AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(Lane, 2, is_intersection, false)
AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(Lane, 3, is_left_turn, false)
AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(Lane, 4, is_right_turn, false)

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_FEATURE(StopLine, stop_lines)

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_FEATURE(Crosswalk, crosswalks)
AUTOWARE_VECTOR_MAP_REGISTER_GPKG_MEMBER(Crosswalk, 0, width, 0.0)

AUTOWARE_VECTOR_MAP_REGISTER_GPKG_FEATURE(TrafficLight, traffic_lights)

}  // namespace traits
}  // namespace autoware_vector_map

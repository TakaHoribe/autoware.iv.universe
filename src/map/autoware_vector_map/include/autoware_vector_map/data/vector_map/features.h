#pragma once

#include <string>

#include <autoware_vector_map/data/data.h>

namespace autoware_vector_map {
namespace data {

template <class T_Geometry>
struct Feature {
  using GeometryType = T_Geometry;

  Id id;
  T_Geometry geometry;
};

struct IntersectionArea : public Feature<Polygon> {};

struct LaneSection : public Feature<Polygon> {};

struct Lane : public Feature<LineString> {
  Id lane_section_id;
  double width;
  bool is_intersection;
  bool is_left_turn;
  bool is_right_turn;
  bool can_left_lane_change;
  bool can_right_lane_change;
};

struct StopLine : public Feature<LineString> {};

struct Crosswalk : public Feature<LineString> {
  double width;
};

struct TrafficLight : public Feature<LineString> {};

}  // namespace data
}  // namespace autoware_vector_map

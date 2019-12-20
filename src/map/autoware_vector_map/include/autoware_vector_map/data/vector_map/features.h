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
  bool can_left_lane_change;
  bool can_right_lane_change;
  bool is_intersection;
  bool is_left_turn;
  bool is_right_turn;
};

struct StopLine : public Feature<LineString> {
  bool is_reason_rule;
  bool is_reason_crosswalk;
  bool is_reason_traffic_light;
  bool is_reason_standby;
  bool is_reason_virtual;
};

struct Crosswalk : public Feature<LineString> {
  double width;
};

struct TrafficLight : public Feature<LineString> {};

}  // namespace data
}  // namespace autoware_vector_map

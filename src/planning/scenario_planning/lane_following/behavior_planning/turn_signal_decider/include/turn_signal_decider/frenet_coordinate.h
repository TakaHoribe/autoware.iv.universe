#pragma once

#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <vector>

namespace turn_signal_decider {
struct FrenetCoordinate3d {
  double length;
  double distance;
  FrenetCoordinate3d() : length(0), distance(0) {}
};

bool convertToFrenetCoordinate3d(const autoware_planning_msgs::PathWithLaneId& path,
                                 const geometry_msgs::Point& search_point_geom, FrenetCoordinate3d* frenet_coordinate);
bool convertToFrenetCoordinate3d(const std::vector<geometry_msgs::Point>& linestring,
                                 const geometry_msgs::Point& search_point_geom, FrenetCoordinate3d* frenet_coordinate);
}  // namespace turn_signal_decider
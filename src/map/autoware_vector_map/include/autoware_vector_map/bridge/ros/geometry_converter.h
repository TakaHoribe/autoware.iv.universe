#pragma once

#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <autoware_vector_map/data/data.h>

namespace autoware_vector_map {
namespace bridge {
namespace ros {

using autoware_vector_map::data::LinearRing;
using autoware_vector_map::data::LineString;
using autoware_vector_map::data::Point;
using autoware_vector_map::data::Polygon;

template <class T>
struct RosGeometry;

template <>
struct RosGeometry<Point> {
  using type = geometry_msgs::Point;
};

template <>
struct RosGeometry<LineString> {
  using type = std::vector<geometry_msgs::Point>;
};

template <>
struct RosGeometry<LinearRing> {
  using type = std::vector<geometry_msgs::Point>;
};

template <>
struct RosGeometry<Polygon> {
  using type = std::vector<geometry_msgs::Point>;
};

template <class T, class T_Ros = typename RosGeometry<T>::type>
T fromRosGeometry(const T_Ros& ros_geom);

template <class T, class T_Ros = typename RosGeometry<T>::type>
T_Ros toRosGeometry(const T& geom);

}  // namespace ros
}  // namespace bridge
}  // namespace autoware_vector_map

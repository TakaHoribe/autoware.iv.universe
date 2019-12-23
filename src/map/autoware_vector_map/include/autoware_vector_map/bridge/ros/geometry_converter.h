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
struct ros_geometry;

template <class T>
using ros_geometry_t = typename ros_geometry<T>::type;

template <>
struct ros_geometry<Point> {
  using type = geometry_msgs::Point;
};

template <>
struct ros_geometry<LineString> {
  using type = std::vector<geometry_msgs::Point>;
};

template <>
struct ros_geometry<LinearRing> {
  using type = std::vector<geometry_msgs::Point>;
};

template <>
struct ros_geometry<Polygon> {
  using type = std::vector<geometry_msgs::Point>;
};

template <class T, class T_Ros = ros_geometry_t<T>>
T fromRosGeometry(const T_Ros& ros_geom);

template <class T, class T_Ros = ros_geometry_t<T>>
T_Ros toRosGeometry(const T& geom);

}  // namespace ros
}  // namespace bridge
}  // namespace autoware_vector_map

#include <autoware_vector_map/bridge/ros/geometry_converter.h>

namespace autoware_vector_map {
namespace bridge {
namespace ros {

template <>
Point fromRosGeometry<Point>(const geometry_msgs::Point& p) {
  return Point(p.x, p.y, p.z);
}

template <>
LineString fromRosGeometry<LineString>(const std::vector<geometry_msgs::Point>& ros_geom) {
  LineString geom{};

  geom.reserve(ros_geom.size());
  for (const auto& p : ros_geom) {
    geom.emplace_back(p.x, p.y, p.z);
  }

  return geom;
}

template <>
LinearRing fromRosGeometry<LinearRing>(const std::vector<geometry_msgs::Point>& ros_geom) {
  LinearRing geom{};

  geom.reserve(ros_geom.size());
  for (const auto& p : ros_geom) {
    geom.emplace_back(p.x, p.y, p.z);
  }

  return geom;
}

template <>
Polygon fromRosGeometry<Polygon>(const std::vector<geometry_msgs::Point>& ros_geom) {
  Polygon geom{};

  geom.exterior = fromRosGeometry<LinearRing>(ros_geom);

  return geom;
}

template <>
geometry_msgs::Point toRosGeometry<Point>(const Point& geom) {
  geometry_msgs::Point p;

  p.x = geom.x();
  p.y = geom.y();
  p.z = geom.z();

  return p;
}

template <>
std::vector<geometry_msgs::Point> toRosGeometry<LineString>(const LineString& geom) {
  std::vector<geometry_msgs::Point> ros_geom{};

  for (const auto& p : geom) {
    ros_geom.push_back(toRosGeometry<Point>(p));
  }

  return ros_geom;
}

template <>
std::vector<geometry_msgs::Point> toRosGeometry<LinearRing>(const LinearRing& geom) {
  std::vector<geometry_msgs::Point> ros_geom{};

  for (const auto& p : geom) {
    ros_geom.push_back(toRosGeometry<Point>(p));
  }

  return ros_geom;
}

template <>
std::vector<geometry_msgs::Point> toRosGeometry<Polygon>(const Polygon& geom) {
  std::vector<geometry_msgs::Point> ros_geom{};

  for (const auto& p : geom.exterior) {
    ros_geom.push_back(toRosGeometry<Point>(p));
  }

  return ros_geom;
}
}  // namespace ros
}  // namespace bridge
}  // namespace autoware_vector_map

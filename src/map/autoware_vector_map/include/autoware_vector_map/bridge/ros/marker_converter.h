#pragma once

#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include <autoware_vector_map/data/data.h>

namespace autoware_vector_map {
namespace bridge {
namespace ros {

template <class T>
visualization_msgs::Marker createMarker(const char* frame_id, const char* ns, const int32_t id,
                                        const T& geometry, const std_msgs::ColorRGBA& color);

}  // namespace ros
}  // namespace bridge
}  // namespace autoware_vector_map

inline geometry_msgs::Point createMarkerPosition(const double x, const double y, const double z) {
  geometry_msgs::Point point;

  point.x = x;
  point.y = y;
  point.z = z;

  return point;
}

inline geometry_msgs::Quaternion createMarkerOrientation(const double x, const double y,
                                                         const double z, const double w) {
  geometry_msgs::Quaternion quaternion;

  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;

  return quaternion;
}

inline geometry_msgs::Vector3 createMarkerScale(const double x, const double y, const double z) {
  geometry_msgs::Vector3 scale;

  scale.x = x;
  scale.y = y;
  scale.z = z;

  return scale;
}

inline std_msgs::ColorRGBA createMarkerColor(const float r, const float g, const float b,
                                             const float a) {
  std_msgs::ColorRGBA color;

  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  return color;
}

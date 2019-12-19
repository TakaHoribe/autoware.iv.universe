#pragma once

#include <visualization_msgs/MarkerArray.h>

geometry_msgs::Point createMarkerPosition(const double x, const double y, const double z) {
  geometry_msgs::Point point;

  point.x = x;
  point.y = y;
  point.z = z;

  return point;
}

geometry_msgs::Quaternion createMarkerOrientation(const double x, const double y, const double z,
                                                  const double w) {
  geometry_msgs::Quaternion quaternion;

  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;

  return quaternion;
}

geometry_msgs::Vector3 createMarkerScale(const double x, const double y, const double z) {
  geometry_msgs::Vector3 scale;

  scale.x = x;
  scale.y = y;
  scale.z = z;

  return scale;
}

std_msgs::ColorRGBA createMarkerColor(const float r, const float g, const float b, const float a) {
  std_msgs::ColorRGBA color;

  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  return color;
}

template <class T>
visualization_msgs::Marker initMarker(const char* ns, const int32_t id, const int32_t type,
                                      const std_msgs::ColorRGBA& color) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = ns;
  marker.id = id;
  marker.type = type;
  marker.action = visualization_msgs::Marker::ADD;
  marker.lifetime = ros::Duration(0);

  marker.pose.position = createMarkerPosition(0.0, 0.0, 0.0);
  marker.pose.orientation = createMarkerOrientation(0.0, 0.0, 0.0, 1.0);
  marker.scale = createMarkerScale(1.0, 1.0, 1.0);
  marker.color = color;

  return marker;
}

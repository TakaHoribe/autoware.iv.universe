#pragma once

#include <visualization_msgs/MarkerArray.h>

inline geometry_msgs::Point createMarkerPosition(double x, double y, double z) {
  geometry_msgs::Point point;

  point.x = x;
  point.y = y;
  point.z = z;

  return point;
}

inline geometry_msgs::Quaternion createMarkerOrientation(double x, double y, double z, double w) {
  geometry_msgs::Quaternion quaternion;

  quaternion.x = x;
  quaternion.y = y;
  quaternion.z = z;
  quaternion.w = w;

  return quaternion;
}

inline geometry_msgs::Vector3 createMarkerScale(double x, double y, double z) {
  geometry_msgs::Vector3 scale;

  scale.x = x;
  scale.y = y;
  scale.z = z;

  return scale;
}

inline std_msgs::ColorRGBA createMarkerColor(float r, float g, float b, float a) {
  std_msgs::ColorRGBA color;

  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;

  return color;
}

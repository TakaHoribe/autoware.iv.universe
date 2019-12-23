#pragma once

#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace autoware_vector_map {
namespace data {

// Eigen
using Position = Eigen::Vector3d;
using Orientation = Eigen::Quaterniond;

struct Pose {
  Position position;
  Orientation orientation;
};

struct Point : public Position {
  Point() = default;
  Point(const double x, const double y, const double z = 0.0) : Position(x, y, z) {}
};

struct LineString : public std::vector<Point> {
  using std::vector<Point>::vector;
};

struct LinearRing : public LineString {
  using LineString::LineString;
};

struct Polygon {
  LinearRing exterior;
  std::vector<LinearRing> interiors;
};

}  // namespace data
}  // namespace autoware_vector_map

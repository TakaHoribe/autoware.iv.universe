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

using Point = Position;

using LineString = std::vector<Point>;

struct LinearRing : public LineString {};

struct Polygon {
  LinearRing exterior;
  std::vector<LinearRing> interiors;
};

}  // namespace data
}  // namespace autoware_vector_map

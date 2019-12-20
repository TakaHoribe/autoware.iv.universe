#pragma once

#include <vector>

#include <autoware_vector_map/core.h>

namespace autoware_vector_map {
namespace geometry {

std::vector<Point3d> intersection(const LineString3d& geom1, const LineString3d& geom2);

std::vector<Point2d> intersection(const LineString2d& geom1, const LineString2d& geom2);
std::vector<Polygon2d> intersection(const LinearRing2d& geom1, const LinearRing2d& geom2);
std::vector<Polygon2d> intersection(const Polygon2d& geom1, const Polygon2d& geom2);

}  // namespace geometry
}  // namespace autoware_vector_map

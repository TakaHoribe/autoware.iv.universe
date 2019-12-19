#pragma once

#include <string>

#include <ogrsf_frmts.h>

#include <autoware_vector_map/data/data.h>

namespace autoware_vector_map {
namespace bridge {
namespace ogr {

using autoware_vector_map::data::LinearRing;
using autoware_vector_map::data::LineString;
using autoware_vector_map::data::Point;
using autoware_vector_map::data::Polygon;

template <class T>
struct OgrGeometry;

template <>
struct OgrGeometry<Point> {
  using type = OGRPoint;
};

template <>
struct OgrGeometry<LineString> {
  using type = OGRLineString;
};

template <>
struct OgrGeometry<LinearRing> {
  using type = OGRLinearRing;
};

template <>
struct OgrGeometry<Polygon> {
  using type = OGRPolygon;
};

template <class T, class T_Ogr = typename OgrGeometry<T>::type>
T fromOgrGeometry(T_Ogr* ogr_geom);

template <class T, class T_Ogr = typename OgrGeometry<T>::type>
T_Ogr toOgrGeometry(const T& geom);

}  // namespace ogr
}  // namespace bridge
}  // namespace autoware_vector_map

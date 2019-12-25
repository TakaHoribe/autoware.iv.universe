#pragma once

#include <string>
#include <vector>

#include <ogrsf_frmts.h>

#include <autoware_vector_map/core.h>

namespace autoware_vector_map {
namespace bridge {

template <class T>
struct ogr_geometry;

template <>
struct ogr_geometry<Point3d> {
  using type = OGRPoint;
};

template <>
struct ogr_geometry<LineString3d> {
  using type = OGRLineString;
};

template <>
struct ogr_geometry<LinearRing3d> {
  using type = OGRLinearRing;
};

template <>
struct ogr_geometry<Polygon3d> {
  using type = OGRPolygon;
};

template <>
struct ogr_geometry<std::vector<Point3d>> {
  using type = OGRMultiPoint;
};

template <>
struct ogr_geometry<std::vector<LineString3d>> {
  using type = OGRMultiLineString;
};

template <>
struct ogr_geometry<std::vector<Polygon3d>> {
  using type = OGRMultiPolygon;
};

template <class T>
using ogr_geometry_t = typename ogr_geometry<T>::type;

template <class T>
struct ogr_geometry_name;

template <>
struct ogr_geometry_name<OGRPoint> {
  static constexpr const char* value = "POINT";
};

template <>
struct ogr_geometry_name<OGRLineString> {
  static constexpr const char* value = "LINESTRING";
};

template <>
struct ogr_geometry_name<OGRLinearRing> {
  static constexpr const char* value = "LINEARRING";
};

template <>
struct ogr_geometry_name<OGRPolygon> {
  static constexpr const char* value = "POLYGON";
};

template <>
struct ogr_geometry_name<OGRMultiPoint> {
  static constexpr const char* value = "MULTIPOINT";
};

template <>
struct ogr_geometry_name<OGRMultiLineString> {
  static constexpr const char* value = "MULTILINESTRING";
};

template <>
struct ogr_geometry_name<OGRMultiPolygon> {
  static constexpr const char* value = "MULTIPOLYGON";
};

template <class T, class T_Ogr = ogr_geometry_t<T>>
T fromOgrGeometry(T_Ogr* ogr_geom);

template <class T, class T_Ogr = ogr_geometry_t<T>>
T_Ogr toOgrGeometry(const T& geom);

}  // namespace bridge
}  // namespace autoware_vector_map

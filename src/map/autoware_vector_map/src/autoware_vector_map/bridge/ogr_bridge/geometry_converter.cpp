#include <autoware_vector_map/bridge/ogr/geometry_converter.h>
#include <iostream>
namespace autoware_vector_map {
namespace bridge {
namespace ogr {

template <>
Point fromOgrGeometry<Point>(OGRPoint* ogr_geom) {
  return Point(ogr_geom->getX(), ogr_geom->getY(), ogr_geom->getZ());
}

template <>
LineString fromOgrGeometry<LineString>(OGRLineString* ogr_geom) {
  LineString geom{};

  geom.reserve(ogr_geom->getNumPoints());
  for (int i = 0; i < ogr_geom->getNumPoints(); ++i) {
    geom.emplace_back(ogr_geom->getX(i), ogr_geom->getY(i), ogr_geom->getZ(i));
  }

  return geom;
}

template <>
LinearRing fromOgrGeometry<LinearRing>(OGRLinearRing* ogr_geom) {
  LinearRing geom{};

  geom.reserve(ogr_geom->getNumPoints());
  for (int i = 0; i < ogr_geom->getNumPoints(); ++i) {
    geom.emplace_back(ogr_geom->getX(i), ogr_geom->getY(i), ogr_geom->getZ(i));
  }

  return geom;
}

template <>
Polygon fromOgrGeometry<Polygon>(OGRPolygon* ogr_geom) {
  Polygon geom{};

  geom.exterior = fromOgrGeometry<LinearRing>(ogr_geom->getExteriorRing());

  geom.interiors.reserve(ogr_geom->getNumInteriorRings());
  for (int i = 0; i < ogr_geom->getNumInteriorRings(); ++i) {
    geom.interiors.push_back(fromOgrGeometry<LinearRing>(ogr_geom->getInteriorRing(i)));
  }

  return geom;
}

template <>
OGRPoint toOgrGeometry<Point>(const Point& geom) {
  return OGRPoint(geom.x(), geom.y(), geom.z());
}

template <>
OGRLineString toOgrGeometry<LineString>(const LineString& geom) {
  OGRLineString ogr_geom{};

  for (const auto& p : geom) {
    ogr_geom.addPoint(p.x(), p.y(), p.z());
  }

  return ogr_geom;
}

template <>
OGRLinearRing toOgrGeometry<LinearRing>(const LinearRing& geom) {
  OGRLinearRing ogr_geom{};

  for (const auto& p : geom) {
    ogr_geom.addPoint(p.x(), p.y(), p.z());
  }

  return ogr_geom;
}

template <>
OGRPolygon toOgrGeometry<Polygon>(const Polygon& geom) {
  OGRPolygon ogr_geom{};

  OGRLinearRing* exterior_ring = new OGRLinearRing();
  *exterior_ring = toOgrGeometry(geom.exterior);
  ogr_geom.addRingDirectly(exterior_ring);

  for (size_t i = 0; i < geom.interiors.size(); ++i) {
    OGRLinearRing* interior_ring = new OGRLinearRing();
    *interior_ring = toOgrGeometry(geom.interiors.at(i));
    ogr_geom.addRingDirectly(interior_ring);
  }

  return ogr_geom;
}

}  // namespace ogr
}  // namespace bridge
}  // namespace autoware_vector_map

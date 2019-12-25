#include <autoware_vector_map/bridge/ros/marker_converter.h>

#include <autoware_vector_map/bridge/ros/geometry_converter.h>
#include <autoware_vector_map/core.h>

#include "earcut.hpp"

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace {

using autoware_vector_map::LinearRing3d;
using autoware_vector_map::LineString3d;
using autoware_vector_map::Point3d;
using autoware_vector_map::Polygon3d;

using Triangle = std::array<Point3d, 3>;
std::vector<Triangle> polygon2triangles(const Polygon3d& polygon) {
  using MapboxPoint = std::array<double, 3>;
  using MapboxLinearRing = std::vector<MapboxPoint>;
  using MapboxPolygon = std::vector<MapboxLinearRing>;

  const auto toMapboxLinearRing = [](const LinearRing3d& linear_ring) {
    MapboxLinearRing mapbox_linear_ring;
    for (const auto& p : linear_ring) {
      mapbox_linear_ring.push_back({p.x(), p.y(), p.z()});
    }
    return mapbox_linear_ring;
  };

  // Create mapbox polygon
  MapboxPolygon mapbox_polygon;
  mapbox_polygon.push_back(toMapboxLinearRing(polygon.exterior));
  for (const auto& interior : polygon.interiors) {
    mapbox_polygon.push_back(toMapboxLinearRing(interior));
  }

  // Call earcut
  using N = uint32_t;
  std::vector<N> indices = mapbox::earcut<N>(mapbox_polygon);

  // Calculate number of triangles
  assert(indices.size() % 3 == 0);
  const size_t num_triangles = indices.size() / 3;

  // Flatten points
  std::vector<Point3d> flattened_points;
  for (const auto& mapbox_linear_ring : mapbox_polygon) {
    for (const auto& mapbox_point : mapbox_linear_ring) {
      flattened_points.emplace_back(mapbox_point.at(0), mapbox_point.at(1), mapbox_point.at(2));
    }
  }

  // Convert indices to triangles
  std::vector<Triangle> triangles;
  triangles.reserve(num_triangles);
  for (size_t i = 0; i < num_triangles; ++i) {
    Triangle triangle;

    triangle[0] = flattened_points.at(indices.at(3 * i + 0));
    triangle[1] = flattened_points.at(indices.at(3 * i + 1));
    triangle[2] = flattened_points.at(indices.at(3 * i + 2));

    triangles.push_back(triangle);
  }

  return triangles;
}

visualization_msgs::Marker initMarker(const char* frame_id, const char* ns, const int32_t id,
                                      const int32_t type, const std_msgs::ColorRGBA& color) {
  visualization_msgs::Marker marker;

  marker.header.frame_id = frame_id;
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

}  // namespace

namespace autoware_vector_map {
namespace bridge {
namespace ros {

template <>
visualization_msgs::Marker createMarker<Point3d>(const char* frame_id, const char* ns,
                                                 const int32_t id, const Point3d& geometry,
                                                 const std_msgs::ColorRGBA& color) {
  auto marker = initMarker(frame_id, ns, id, Marker::SPHERE, color);

  marker.pose.position = toRosGeometry(geometry);

  marker.points.push_back(toRosGeometry(geometry));
  marker.colors.push_back(marker.color);

  return marker;
}

template <>
visualization_msgs::Marker createMarker<LineString3d>(const char* frame_id, const char* ns,
                                                      const int32_t id,
                                                      const LineString3d& geometry,
                                                      const std_msgs::ColorRGBA& color) {
  auto marker = initMarker(frame_id, ns, id, Marker::LINE_STRIP, color);

  for (const auto& point : geometry) {
    marker.points.push_back(toRosGeometry(point));
    marker.colors.push_back(marker.color);
  }

  return marker;
}

template <>
visualization_msgs::Marker createMarker<Polygon3d>(const char* frame_id, const char* ns,
                                                   const int32_t id, const Polygon3d& geometry,
                                                   const std_msgs::ColorRGBA& color) {
  auto marker = initMarker(frame_id, ns, id, Marker::TRIANGLE_LIST, color);

  for (const auto& triangle : polygon2triangles(geometry)) {
    for (const auto& point : triangle) {
      marker.points.push_back(toRosGeometry(point));
      marker.colors.push_back(marker.color);
    }
  }

  return marker;
}

template <>
visualization_msgs::Marker createMarker<LinearRing3d>(const char* frame_id, const char* ns,
                                                      const int32_t id,
                                                      const LinearRing3d& geometry,
                                                      const std_msgs::ColorRGBA& color) {
  Polygon3d polygon{};
  polygon.exterior = geometry;
  return createMarker(frame_id, ns, id, polygon, color);
}

template <>
visualization_msgs::Marker createMarker<Point2d>(const char* frame_id, const char* ns,
                                                 const int32_t id, const Point2d& geometry,
                                                 const std_msgs::ColorRGBA& color) {
  return createMarker(frame_id, ns, id, geometry.to_3d(), color);
}

template <>
visualization_msgs::Marker createMarker<LineString2d>(const char* frame_id, const char* ns,
                                                      const int32_t id,
                                                      const LineString2d& geometry,
                                                      const std_msgs::ColorRGBA& color) {
  return createMarker(frame_id, ns, id, geometry.to_3d(), color);
}

template <>
visualization_msgs::Marker createMarker<LinearRing2d>(const char* frame_id, const char* ns,
                                                      const int32_t id,
                                                      const LinearRing2d& geometry,
                                                      const std_msgs::ColorRGBA& color) {
  return createMarker(frame_id, ns, id, geometry.to_3d(), color);
}

template <>
visualization_msgs::Marker createMarker<Polygon2d>(const char* frame_id, const char* ns,
                                                   const int32_t id, const Polygon2d& geometry,
                                                   const std_msgs::ColorRGBA& color) {
  return createMarker(frame_id, ns, id, geometry.to_3d(), color);
}

}  // namespace ros
}  // namespace bridge
}  // namespace autoware_vector_map

#include "vector_map_visualizer_node.h"

#include <algorithm>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include <autoware_vector_map/autoware_vector_map.h>
#include <autoware_vector_map/bridge/ros/geometry_converter.h>

#include "earcut.hpp"
#include "marker_helper.h"

namespace avm = autoware_vector_map::data;
using autoware_vector_map::bridge::ros::toRosGeometry;
using autoware_vector_map::io::gpkg_loader::GpkgLoader;
using autoware_vector_map::traits::gpkg_content;

using autoware_vector_map_msgs::BinaryGpkgMap;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

namespace {

using Triangle = std::array<avm::Point, 3>;
std::vector<Triangle> polygon2triangles(const avm::Polygon& polygon) {
  using MapboxPoint = std::array<double, 3>;
  using MapboxLinearRing = std::vector<MapboxPoint>;
  using MapboxPolygon = std::vector<MapboxLinearRing>;

  const auto toMapboxLinearRing = [](const avm::LinearRing& linear_ring) {
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
  std::vector<avm::Point> flattened_points;
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

template <class T,
          std::enable_if_t<std::is_same<T, avm::LineString>::value, std::nullptr_t> = nullptr>
Marker createMarker(const char* ns, const avm::Id id, const T& geometry,
                    const std_msgs::ColorRGBA& color) {
  auto marker = initMarker<T>(ns, id, Marker::LINE_STRIP, color);

  for (const auto& point : geometry) {
    marker.points.push_back(toRosGeometry(point));
    marker.colors.push_back(marker.color);
  }

  return marker;
}

template <class T, std::enable_if_t<std::is_same<T, avm::Polygon>::value, std::nullptr_t> = nullptr>
Marker createMarker(const char* ns, const avm::Id id, const T& geometry,
                    const std_msgs::ColorRGBA& color) {
  auto marker = initMarker<T>(ns, id, Marker::TRIANGLE_LIST, color);

  for (const auto& triangle : polygon2triangles(geometry)) {
    for (const auto& point : triangle) {
      marker.points.push_back(toRosGeometry(point));
      marker.colors.push_back(marker.color);
    }
  }

  return marker;
}

template <class T,
          std::enable_if_t<std::is_same<T, avm::LinearRing>::value, std::nullptr_t> = nullptr>
Marker createMarker(const char* ns, const int32_t id, const T& geometry,
                    const std_msgs::ColorRGBA& color) {
  avm::Polygon polygon{};
  polygon.exterior = geometry;
  return createMarker(ns, id, polygon, color);
}

template <class T>
Marker createMarker(const char* ns, const T& feature, const std_msgs::ColorRGBA& color) {
  return createMarker(ns, static_cast<int32_t>(feature.id), feature.geometry, color);
}

template <class T>
void addMarkers(GpkgLoader* gpkg_loader, MarkerArray* marker_array,
                const std_msgs::ColorRGBA& color,
                const std::function<void(const T&, Marker*)>& post_process = nullptr,
                const char* ns = gpkg_content<T>::class_name()) {
  const auto features = gpkg_loader->getAllFeatures<T>();
  for (const auto& feature : *features) {
    auto marker = createMarker<T>(ns, feature, color);

    if (post_process) {
      post_process(feature, &marker);
    }

    marker_array->markers.push_back(marker);
  }
}

MarkerArray createMarkers(const std::vector<uint8_t>& bin_data) {
  GpkgLoader gpkg_loader(bin_data);

  MarkerArray marker_array;

  addMarkers<avm::IntersectionArea>(&gpkg_loader, &marker_array,
                                    createMarkerColor(0.7, 0.0, 0.0, 0.5));

  addMarkers<avm::LaneSection>(&gpkg_loader, &marker_array, createMarkerColor(0.0, 0.7, 0.0, 0.5));

  addMarkers<avm::Lane>(
      &gpkg_loader, &marker_array, createMarkerColor(0.2, 0.7, 0.7, 0.5),
      [](const auto& f, Marker* marker) { marker->scale = createMarkerScale(f.width, 0.0, 0.0); },
      "LaneArea");

  addMarkers<avm::Lane>(
      &gpkg_loader, &marker_array, createMarkerColor(0.0, 0.0, 0.0, 1.0),
      [](const auto& f, Marker* marker) { marker->scale = createMarkerScale(0.5, 0.0, 0.0); },
      "LaneCenterLine");

  addMarkers<avm::StopLine>(
      &gpkg_loader, &marker_array, createMarkerColor(1.0, 0.0, 0.0, 1.0),
      [](const auto& f, Marker* marker) { marker->scale = createMarkerScale(0.5, 0.0, 0.0); });

  addMarkers<avm::Crosswalk>(
      &gpkg_loader, &marker_array, createMarkerColor(0.0, 1.0, 0.0, 1.0),
      [](const auto& f, Marker* marker) { marker->scale = createMarkerScale(f.width, 0.0, 0.0); });

  return marker_array;
}

}  // namespace

void VectorMapVisualizerNode::onBinaryGpkgMap(
    const autoware_vector_map_msgs::BinaryGpkgMap::ConstPtr& binary_gpkg_map) {
  std::vector<uint8_t> bin_data;
  bin_data.resize(binary_gpkg_map->data.size());

  std::copy(binary_gpkg_map->data.begin(), binary_gpkg_map->data.end(), bin_data.begin());

  const auto marker_array = createMarkers(bin_data);
  pub_marker_array_.publish(marker_array);
}

VectorMapVisualizerNode::VectorMapVisualizerNode() : nh_(""), private_nh_("~") {
  pub_marker_array_ = nh_.advertise<MarkerArray>("autoware_vector_map_viz", 1, true);

  sub_binary_gpkg_map_ = nh_.subscribe("autoware_vector_map/binary_gpkg_map", 1,
                                       &VectorMapVisualizerNode::onBinaryGpkgMap, this);
}

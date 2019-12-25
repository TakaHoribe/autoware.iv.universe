#include "vector_map_visualizer_node.h"

#include <algorithm>
#include <vector>

#include <visualization_msgs/MarkerArray.h>

#include <autoware_vector_map/bridge/ros/marker_converter.h>
#include <autoware_vector_map/io/gpkg_loader.h>

namespace avm = autoware_vector_map;
using autoware_vector_map::bridge::createMarker;
using autoware_vector_map::bridge::createMarkerColor;
using autoware_vector_map::bridge::createMarkerScale;
using autoware_vector_map::io::GpkgLoader;
using autoware_vector_map::traits::gpkg_content;

using autoware_vector_map_msgs::BinaryGpkgMap;

using visualization_msgs::Marker;
using visualization_msgs::MarkerArray;

template <class T>
void addMarkers(GpkgLoader* gpkg_loader, MarkerArray* marker_array,
                const std_msgs::ColorRGBA& color,
                const std::function<void(const T&, Marker*)>& post_process = nullptr,
                const char* ns = gpkg_content<T>::class_name()) {
  const auto features = gpkg_loader->getAllFeatures<T>();
  for (const auto& feature : *features) {
    auto marker =
        createMarker("map", ns, static_cast<int32_t>(feature.id), feature.geometry, color);

    if (post_process) {
      post_process(feature, &marker);
    }

    marker_array->markers.push_back(marker);
  }
}

MarkerArray createAllMarkers(const std::vector<uint8_t>& bin_data) {
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

void VectorMapVisualizerNode::onBinaryGpkgMap(
    const autoware_vector_map_msgs::BinaryGpkgMap::ConstPtr& binary_gpkg_map) {
  std::vector<uint8_t> bin_data;
  bin_data.resize(binary_gpkg_map->data.size());

  std::copy(binary_gpkg_map->data.begin(), binary_gpkg_map->data.end(), bin_data.begin());

  const auto marker_array = createAllMarkers(bin_data);
  pub_marker_array_.publish(marker_array);
}

VectorMapVisualizerNode::VectorMapVisualizerNode() : nh_(""), private_nh_("~") {
  pub_marker_array_ = nh_.advertise<MarkerArray>("autoware_vector_map/vizualization", 1, true);

  sub_binary_gpkg_map_ = nh_.subscribe("autoware_vector_map/binary_gpkg_map", 1,
                                       &VectorMapVisualizerNode::onBinaryGpkgMap, this);
}

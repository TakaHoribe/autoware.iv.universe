#include "vector_map_loader_node.h"

#include <string>

#include <autoware_vector_map/io/gpkg_loader.h>
#include <autoware_vector_map_msgs/BinaryGpkgMap.h>

using autoware_vector_map::io::GpkgLoader;
using autoware_vector_map_msgs::BinaryGpkgMap;

VectorMapLoaderNode::VectorMapLoaderNode() : nh_(""), private_nh_("~") {
  pub_binary_gpkg_map_ =
      nh_.advertise<BinaryGpkgMap>("autoware_vector_map/binary_gpkg_map", 1, true);

  std::string vector_map_path;
  private_nh_.getParam("vector_map_path", vector_map_path);

  GpkgLoader gpkg_loader(vector_map_path.c_str());
  const auto bin_data = gpkg_loader.toBinary();

  BinaryGpkgMap binary_gpkg_map;
  binary_gpkg_map.header.frame_id = "";
  binary_gpkg_map.header.stamp = ros::Time::now();
  binary_gpkg_map.header.frame_id = "map";

  binary_gpkg_map.format_version = "";
  binary_gpkg_map.map_version = "";

  binary_gpkg_map.data.clear();
  binary_gpkg_map.data.assign(bin_data.begin(), bin_data.end());

  pub_binary_gpkg_map_.publish(binary_gpkg_map);
}

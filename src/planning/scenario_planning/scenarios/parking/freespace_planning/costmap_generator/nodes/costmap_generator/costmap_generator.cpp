/*
 *  Copyright (c) 2018, Nagoya University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ********************/

#include <costmap_generator/costmap_generator.h>
#include <costmap_generator/object_map_utils.hpp>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/visualization/visualization.h>

namespace {

// Convert from Point32 to Point
std::vector<geometry_msgs::Point> poly2vector(const geometry_msgs::Polygon& poly) {
  std::vector<geometry_msgs::Point> ps;
  for (const auto& p32 : poly.points) {
    geometry_msgs::Point p;
    p.x = p32.x;
    p.y = p32.y;
    p.z = p32.z;
    ps.push_back(p);
  }
  return ps;
}

}  // namespace

CostmapGenerator::CostmapGenerator() : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_) {}

void CostmapGenerator::init() {
  private_nh_.param<std::string>("costmap_frame", costmap_frame_, "base_link");
  private_nh_.param<std::string>("map_frame", map_frame_, "map");
  private_nh_.param<double>("grid_min_value", grid_min_value_, 0.0);
  private_nh_.param<double>("grid_max_value", grid_max_value_, 1.0);
  private_nh_.param<double>("grid_resolution", grid_resolution_, 0.2);
  private_nh_.param<double>("grid_length_x", grid_length_x_, 50);
  private_nh_.param<double>("grid_length_y", grid_length_y_, 30);
  private_nh_.param<double>("grid_position_x", grid_position_x_, 20);
  private_nh_.param<double>("grid_position_y", grid_position_y_, 0);
  private_nh_.param<double>("maximum_lidar_height_thres", maximum_lidar_height_thres_, 0.3);
  private_nh_.param<double>("minimum_lidar_height_thres", minimum_lidar_height_thres_, -2.2);
  private_nh_.param<bool>("use_objects", use_objects_, true);
  private_nh_.param<bool>("use_points", use_points_, true);
  private_nh_.param<bool>("use_wayarea", use_wayarea_, true);
  private_nh_.param<double>("expand_polygon_size", expand_polygon_size_, 1.0);
  private_nh_.param<int>("size_of_expansion_kernel", size_of_expansion_kernel_, 9);

  initGridmap();
}

void CostmapGenerator::run() {
  pub_costmap_ = private_nh_.advertise<grid_map_msgs::GridMap>("output/costmap", 1);
  pub_occupancy_grid_ = private_nh_.advertise<nav_msgs::OccupancyGrid>("output/occupancy_grid", 1);

  sub_objects_ =
      private_nh_.subscribe("input/objects", 1, &CostmapGenerator::objectsCallback, this);

  sub_points_ =
      private_nh_.subscribe("input/points_no_ground", 1, &CostmapGenerator::pointsCallback, this);

  sub_lanelet_bin_map_ = private_nh_.subscribe("input/lanelet_map_bin", 1,
                                               &CostmapGenerator::laneletBinMapCallback, this);
}

void CostmapGenerator::loadRoadAreasFromLaneletMap(
    const lanelet::LaneletMapPtr lanelet_map,
    std::vector<std::vector<geometry_msgs::Point>>* area_points) {
  // use all lanelets in map of subtype road to give way area
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map);
  lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);

  // convert lanelets to polygons and put into area_points array
  for (const auto& ll : road_lanelets) {
    geometry_msgs::Polygon poly;
    lanelet::visualization::lanelet2Polygon(ll, &poly);
    area_points->push_back(poly2vector(poly));
  }
}

void CostmapGenerator::loadParkingAreasFromLaneletMap(
    const lanelet::LaneletMapPtr lanelet_map,
    std::vector<std::vector<geometry_msgs::Point>>* area_points) {
  // Parking lots
  lanelet::ConstPolygons3d all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map);
  for (const auto& ll_poly : all_parking_lots) {
    geometry_msgs::Polygon poly;
    lanelet::utils::conversion::toGeomMsgPoly(ll_poly, &poly);
    area_points->push_back(poly2vector(poly));
  }

  // Parking spaces
  lanelet::ConstLineStrings3d all_parking_spaces =
      lanelet::utils::query::getAllParkingSpaces(lanelet_map);
  for (const auto& parking_space : all_parking_spaces) {
    lanelet::ConstPolygon3d ll_poly;
    lanelet::utils::lineStringWithWidthToPolygon(parking_space, &ll_poly);

    geometry_msgs::Polygon poly;
    lanelet::utils::conversion::toGeomMsgPoly(ll_poly, &poly);
    area_points->push_back(poly2vector(poly));
  }
}

void CostmapGenerator::laneletBinMapCallback(const autoware_lanelet2_msgs::MapBin& msg) {
  if (!use_wayarea_) {
    return;
  }

  lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_);
  loaded_lanelet_map_ = true;
  loadRoadAreasFromLaneletMap(lanelet_map_, &area_points_);
  loadParkingAreasFromLaneletMap(lanelet_map_, &area_points_);
}

void CostmapGenerator::objectsCallback(
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr& in_objects) {
  if (!use_objects_) {
    return;
  }

  costmap_[LayerName::objects] = generateObjectsCostmap(in_objects);
  costmap_[LayerName::wayarea] = generateWayAreaCostmap();
  costmap_[LayerName::combined] = generateCombinedCostmap();

  std_msgs::Header in_header = in_objects->header;
  publishRosMsg(costmap_, in_header);
}

void CostmapGenerator::pointsCallback(const sensor_msgs::PointCloud2::ConstPtr& in_points_msg) {
  if (!use_points_) {
    return;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr in_points(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*in_points_msg, *in_points);
  costmap_[LayerName::points] = generatePointsCostmap(in_points);
  costmap_[LayerName::wayarea] = generateWayAreaCostmap();
  costmap_[LayerName::combined] = generateCombinedCostmap();

  std_msgs::Header in_header = in_points_msg->header;
  publishRosMsg(costmap_, in_header);
}

void CostmapGenerator::initGridmap() {
  costmap_.setFrameId(costmap_frame_);
  costmap_.setGeometry(grid_map::Length(grid_length_x_, grid_length_y_), grid_resolution_,
                       grid_map::Position(grid_position_x_, grid_position_y_));

  costmap_.add(LayerName::points, grid_min_value_);
  costmap_.add(LayerName::objects, grid_min_value_);
  costmap_.add(LayerName::wayarea, grid_min_value_);
  costmap_.add(LayerName::combined, grid_min_value_);
}

grid_map::Matrix CostmapGenerator::generatePointsCostmap(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& in_points) {
  grid_map::Matrix points_costmap = points2costmap_.makeCostmapFromPoints(
      maximum_lidar_height_thres_, minimum_lidar_height_thres_, grid_min_value_, grid_max_value_,
      costmap_, LayerName::points, in_points);
  return points_costmap;
}

autoware_perception_msgs::DynamicObjectArray::ConstPtr transformObjects(
    const tf2_ros::Buffer& tf_buffer,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr& in_objects,
    const std::string& target_frame_id, const std::string& src_frame_id) {
  auto objects = new autoware_perception_msgs::DynamicObjectArray();
  *objects = *in_objects;
  objects->header.frame_id = target_frame_id;

  geometry_msgs::TransformStamped objects2costmap;
  try {
    objects2costmap =
        tf_buffer.lookupTransform(target_frame_id, src_frame_id, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("[costmap_generator] %s", ex.what());
  }

  for (auto& object : objects->objects) {
    auto& pose = object.state.pose_covariance.pose;
    tf2::doTransform(pose, pose, objects2costmap);
  }

  return autoware_perception_msgs::DynamicObjectArray::ConstPtr(objects);
}

grid_map::Matrix CostmapGenerator::generateObjectsCostmap(
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr& in_objects) {
  const auto object_frame = in_objects->header.frame_id;
  const auto transformed_objects =
      transformObjects(tf_buffer_, in_objects, costmap_frame_, object_frame);

  grid_map::Matrix objects_costmap = objects2costmap_.makeCostmapFromObjects(
      costmap_, expand_polygon_size_, size_of_expansion_kernel_, transformed_objects);

  return objects_costmap;
}

grid_map::Matrix CostmapGenerator::generateWayAreaCostmap() {
  grid_map::GridMap lanelet2_costmap = costmap_;
  if (!area_points_.empty()) {
    object_map::FillPolygonAreas(lanelet2_costmap, area_points_, LayerName::wayarea,
                                 grid_max_value_, grid_min_value_, grid_min_value_, grid_max_value_,
                                 costmap_frame_, map_frame_, tf_buffer_);
  }
  return lanelet2_costmap[LayerName::wayarea];
}

grid_map::Matrix CostmapGenerator::generateCombinedCostmap() {
  // assuming combined_costmap is calculated by element wise max operation
  grid_map::GridMap combined_costmap = costmap_;

  combined_costmap[LayerName::combined].setConstant(grid_min_value_);

  combined_costmap[LayerName::combined] =
      combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::points]);

  combined_costmap[LayerName::combined] =
      combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::wayarea]);

  combined_costmap[LayerName::combined] =
      combined_costmap[LayerName::combined].cwiseMax(combined_costmap[LayerName::objects]);

  return combined_costmap[LayerName::combined];
}

void CostmapGenerator::publishRosMsg(const grid_map::GridMap& costmap,
                                     const std_msgs::Header& in_header) {
  nav_msgs::OccupancyGrid out_occupancy_grid;
  grid_map::GridMapRosConverter::toOccupancyGrid(costmap, LayerName::combined, grid_min_value_,
                                                 grid_max_value_, out_occupancy_grid);
  out_occupancy_grid.header = in_header;
  out_occupancy_grid.header.frame_id = costmap_frame_;
  pub_occupancy_grid_.publish(out_occupancy_grid);

  grid_map_msgs::GridMap out_gridmap_msg;
  grid_map::GridMapRosConverter::toMessage(costmap, out_gridmap_msg);
  out_gridmap_msg.info.header = in_header;
  pub_costmap_.publish(out_gridmap_msg);
}

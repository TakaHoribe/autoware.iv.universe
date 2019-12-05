/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <string>
#include <memory>

namespace behavior_planning {

class SelfPoseLinstener
{
public:
  SelfPoseLinstener();
  bool getSelfPose(geometry_msgs::Pose &self_pose, const std_msgs::Header &header);

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

class BehaviorVelocityPlannerNode
{
public:
  BehaviorVelocityPlannerNode();

private:
  /* 
   * ROS
   */
  // publisher and subscriber
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber path_with_lane_id_sub_;
  ros::Subscriber perception_sub_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher path_pub_;
  ros::Publisher debug_viz_pub_;
  //  parameter
  double path_length_;
  // topic cache
  std::shared_ptr<autoware_perception_msgs::DynamicObjectArray> perception_ptr_;
  std::shared_ptr<sensor_msgs::PointCloud2> pointcloud_ptr_;

  /* 
   * Lanelet 
   */
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  /* 
   * Spline 
   */

  void pathWithLaneIdCallback(const autoware_planning_msgs::PathWithLaneId &input_path_msg);
  void perceptionCallback(const autoware_perception_msgs::DynamicObjectArray &input_perception_msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2 &input_pointcloud_msg);
  void mapCallback(const autoware_lanelet2_msgs::MapBin& input_map_msg);
  bool callback(const autoware_planning_msgs::PathWithLaneId &input_path_msg,
                const autoware_perception_msgs::DynamicObjectArray &input_perception_msg,
                const sensor_msgs::PointCloud2 &input_pointcloud_msg,
                autoware_planning_msgs::Path &output_path_msg);
  void publishDebugMarker(const autoware_planning_msgs::Path &path, const ros::Publisher &pub);
  void filterPath(const autoware_planning_msgs::Path &path, autoware_planning_msgs::Path &filtered_path);
  void interporatePath(const autoware_planning_msgs::Path &path, const double length, autoware_planning_msgs::Path &interporated_path);
};
}

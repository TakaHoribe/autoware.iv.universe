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
#pragma once

#include <memory>
#include <string>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <behavior_velocity_planner/planner_data.h>
#include <behavior_velocity_planner/planner_manager.hpp>

namespace behavior_planning {

class BehaviorVelocityPlannerNode {
 public:
  BehaviorVelocityPlannerNode();

 private:
  // node handle
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // tf
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // subscriber
  ros::Subscriber perception_sub_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber vehicle_velocity_sub_;
  ros::Subscriber traffic_light_states_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber path_with_lane_id_sub_;

  void perceptionCallback(const autoware_perception_msgs::DynamicObjectArray::ConstPtr& msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void mapCallback(const autoware_lanelet2_msgs::MapBin::ConstPtr& msg);
  void trafficLightStatesCallback(const autoware_traffic_light_msgs::TrafficLightStateArray::ConstPtr& msg);
  void pathWithLaneIdCallback(const autoware_planning_msgs::PathWithLaneId& input_path_msg);

  // publisher
  ros::Publisher path_pub_;
  ros::Publisher debug_viz_pub_;

  void publishDebugMarker(const autoware_planning_msgs::Path& path, const ros::Publisher& pub);

  //  parameter
  double foward_path_length_;
  double backward_path_length_;

  // member
  PlannerData planner_data_;
  std::shared_ptr<BehaviorVelocityPlannerManager> planner_manager_ptr_;

  // function
  geometry_msgs::PoseStamped getCurrentPose();
};
}  // namespace behavior_planning

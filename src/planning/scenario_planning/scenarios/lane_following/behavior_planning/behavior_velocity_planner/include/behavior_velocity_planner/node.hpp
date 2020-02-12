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

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <behavior_velocity_planner/data_manager.hpp>
#include <behavior_velocity_planner/planner_manager.hpp>

#include <memory>
#include <string>

namespace behavior_planning {

class BehaviorVelocityPlannerNode {
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
  ros::Subscriber vehicle_velocity_sub_;
  ros::Subscriber traffic_light_states_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher path_pub_;
  ros::Publisher debug_viz_pub_;
  //  parameter
  double foward_path_length_;
  double backward_path_length_;

  /*
   * Manager
   */
  std::shared_ptr<BehaviorVelocityPlannerManager> planner_manager_ptr_;

  void pathWithLaneIdCallback(const autoware_planning_msgs::PathWithLaneId& input_path_msg);
  void publishDebugMarker(const autoware_planning_msgs::Path& path, const ros::Publisher& pub);
};
}  // namespace behavior_planning

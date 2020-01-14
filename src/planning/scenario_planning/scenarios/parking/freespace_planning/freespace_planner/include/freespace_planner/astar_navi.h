/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef ASTAR_NAVI_H
#define ASTAR_NAVI_H

#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Scenario.h>
#include <autoware_planning_msgs/Trajectory.h>

#include "astar_search/astar_search.h"

class AstarNavi {
 public:
  AstarNavi();

 private:
  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Publisher trajectory_pub_;
  ros::Publisher debug_pose_array_pub_;

  ros::Subscriber route_sub_;
  ros::Subscriber occupancy_grid_sub_;
  ros::Subscriber scenario_sub_;

  ros::Timer timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // params
  double waypoints_velocity_;  // constant velocity on planned waypoints [km/h]
  double update_rate_;         // replanning and publishing rate [Hz]

  // classes
  AstarSearch astar_;

  // variables
  bool is_active_;
  nav_msgs::OccupancyGrid occupancy_grid_;
  geometry_msgs::PoseStamped current_pose_local_;
  geometry_msgs::PoseStamped current_pose_global_;
  geometry_msgs::PoseStamped goal_pose_local_;
  geometry_msgs::PoseStamped goal_pose_global_;

  bool occupancy_grid_initialized_;
  bool route_initialized_;
  bool current_pose_initialized_;

  // functions, callback
  void onOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
  void onRoute(const autoware_planning_msgs::Route& msg);
  void onScenario(const autoware_planning_msgs::Scenario& msg);

  void onTimer(const ros::TimerEvent& event);

  // fucntions
  geometry_msgs::TransformStamped getTransform(const std::string& from, const std::string& to);
  void publishTrajectory(const AstarWaypoints& astar_waypoints, const double& velocity);
  void publishStopTrajectory();
  void run();
};

#endif

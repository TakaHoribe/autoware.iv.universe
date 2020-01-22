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

#include <deque>
#include <iostream>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
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
  ros::Publisher debug_partial_pose_array_pub_;

  ros::Subscriber route_sub_;
  ros::Subscriber occupancy_grid_sub_;
  ros::Subscriber scenario_sub_;
  ros::Subscriber twist_sub_;

  ros::Timer timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  // params
  double waypoints_velocity_;  // constant velocity on planned waypoints [km/h]
  double update_rate_;         // replanning and publishing rate [Hz]
  double th_stopping_time_sec_;
  double th_stopping_distance_m_;
  double th_stopping_velocity_mps_;

  // variables
  geometry_msgs::PoseStamped current_pose_local_;
  geometry_msgs::PoseStamped current_pose_global_;
  geometry_msgs::PoseStamped goal_pose_local_;
  geometry_msgs::PoseStamped goal_pose_global_;

  autoware_planning_msgs::Trajectory trajectory_;
  std::vector<size_t> reversing_indices_;
  size_t prev_target_index_;
  size_t target_index_;

  autoware_planning_msgs::Route::ConstPtr route_;
  nav_msgs::OccupancyGrid::ConstPtr occupancy_grid_;
  autoware_planning_msgs::Scenario::ConstPtr scenario_;
  geometry_msgs::TwistStamped::ConstPtr twist_;

  std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_buffer_;

  // functions, callback
  void onRoute(const autoware_planning_msgs::Route::ConstPtr& msg);
  void onOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg);
  void onScenario(const autoware_planning_msgs::Scenario::ConstPtr& msg);
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr& msg);

  void onTimer(const ros::TimerEvent& event);

  bool isPlanRequired();
  void planTrajectory();
  void updateTarget();
};

#endif

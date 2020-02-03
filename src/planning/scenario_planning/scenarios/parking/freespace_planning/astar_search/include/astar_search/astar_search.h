/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef ASTER_PLANNER_H
#define ASTER_PLANNER_H

#include <functional>
#include <iostream>
#include <queue>
#include <string>
#include <tuple>
#include <vector>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include "astar_search/astar_util.h"

struct AstarParam {
  // base configs
  bool use_back;      // backward search
  double time_limit;  // planning time limit [msec]

  // robot configs
  double robot_length;            // X [m]
  double robot_width;             // Y [m]
  double robot_base2back;         // base_link to rear [m]
  double minimum_turning_radius;  // [m]]

  // search configs
  int theta_size;                  // descritized angle table size [-]
  double curve_weight;             // curve moving cost [-]
  double reverse_weight;           // backward moving cost [-]
  double lateral_goal_range;       // reaching threshold, lateral error [m]
  double longitudinal_goal_range;  // reaching threshold, longitudinal error [m]
  double angle_goal_range;         // reaching threshold, angle error [deg]

  // costmap configs
  int obstacle_threshold;            // obstacle threshold on grid [-]
  double distance_heuristic_weight;  // obstacle threshold on grid [0,255]
};

class AstarSearch {
 public:
  using StateUpdateTable = std::vector<std::vector<NodeUpdate>>;

  explicit AstarSearch(const AstarParam& astar_param);

  void initializeNodes(const nav_msgs::OccupancyGrid& costmap);
  bool makePlan(const geometry_msgs::Pose& start_pose, const geometry_msgs::Pose& goal_pose);
  bool hasObstacleOnTrajectory(const geometry_msgs::PoseArray& trajectory);

  const nav_msgs::Path& getPath() const { return path_; }
  const AstarWaypoints& getWaypoints() const { return waypoints_; }

 private:
  bool search();
  void setPath(const SimpleNode& goal);
  bool setStartNode(const geometry_msgs::Pose& start_pose);
  bool setGoalNode(const geometry_msgs::Pose& goal_pose);
  bool detectCollision(const SimpleNode& sn);

  bool isOutOfRange(const int index_x, const int index_y);
  bool isObs(const int index_x, const int index_y);
  bool isGoal(const double x, const double y, const double theta);

  AstarParam astar_param_;

  // hybrid astar variables
  StateUpdateTable state_update_table_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> openlist_;
  std::vector<SimpleNode> goallist_;

  // costmap as occupancy grid
  nav_msgs::OccupancyGrid costmap_;

  // pose in costmap frame
  geometry_msgs::PoseStamped start_pose_local_;
  geometry_msgs::PoseStamped goal_pose_local_;
  double goal_yaw_;

  // result path
  nav_msgs::Path path_;
  AstarWaypoints waypoints_;
};

#endif

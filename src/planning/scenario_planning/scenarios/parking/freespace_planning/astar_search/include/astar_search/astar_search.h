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
#include <std_msgs/Header.h>

enum class NodeStatus : uint8_t { None, Open, Closed, Obstacle };

struct IndexXYT {
  int x;
  int y;
  int theta;
};

struct IndexXY {
  int x;
  int y;
};

struct AstarNode {
  IndexXYT index;                        // index
  double x;                              // x
  double y;                              // y
  double theta;                          // theta
  NodeStatus status = NodeStatus::None;  // node status
  double gc = 0;                         // actual cost
  double hc = 0;                         // heuristic cost
  double cost = 0;                       // total cost
  double move_distance = 0;              // actual move distance
  bool is_back;                          // true if the current direction of the vehicle is back
  AstarNode* parent = nullptr;           // parent node

  bool operator>(const AstarNode& right) const { return cost > right.cost; }
};

struct AstarWaypoint {
  geometry_msgs::PoseStamped pose;
  bool is_back = false;
};

struct AstarWaypoints {
  std_msgs::Header header;
  std::vector<AstarWaypoint> waypoints;
};

struct NodeUpdate {
  double shift_x;
  double shift_y;
  double shift_theta;
  double step;
  bool is_curve;
  bool is_back;
};

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
  void setPath(const AstarNode& goal);
  bool setStartNode();
  bool setGoalNode();
  bool detectCollision(const IndexXYT& index);

  bool isOutOfRange(const int index_x, const int index_y);
  bool isObs(const int index_x, const int index_y);
  bool isGoal(const double x, const double y, const double theta);

  AstarParam astar_param_;

  // hybrid astar variables
  StateUpdateTable state_update_table_;
  std::vector<std::vector<std::vector<AstarNode>>> nodes_;
  std::priority_queue<AstarNode, std::vector<AstarNode>, std::greater<AstarNode>> openlist_;
  std::vector<AstarNode> goallist_;

  // costmap as occupancy grid
  nav_msgs::OccupancyGrid costmap_;

  // pose in costmap frame
  geometry_msgs::Pose start_pose_;
  geometry_msgs::Pose goal_pose_;
  double goal_yaw_;

  // result path
  nav_msgs::Path path_;
  AstarWaypoints waypoints_;
};

#endif

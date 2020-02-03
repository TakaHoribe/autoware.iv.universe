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

#ifndef ASTAR_UTIL_H
#define ASTAR_UTIL_H

#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>

enum class Status : uint8_t { NONE, OPEN, CLOSED, OBS };

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
  double x;                      // x
  double y;                      // y
  double theta;                  // theta
  Status status = Status::NONE;  // NONE, OPEN, CLOSED or OBS
  double gc = 0;                 // Actual cost
  double hc = 0;                 // heuristic cost
  double move_distance = 0;      // actual move distance
  bool is_back;                  // true if the current direction of the vehicle is back
  AstarNode* parent = nullptr;   // parent node
};

struct AstarWaypoint {
  geometry_msgs::PoseStamped pose;
  bool is_back = false;
};

struct AstarWaypoints {
  std_msgs::Header header;
  std::vector<AstarWaypoint> waypoints;
};

struct WaveFrontNode {
  IndexXY index;
  double hc;
};

struct NodeUpdate {
  double shift_x;
  double shift_y;
  double rotation;
  double step;
  int index_theta;
  bool is_curve;
  bool is_back;
};

// For open list and goal list
struct SimpleNode {
  IndexXYT index;
  double cost;

  bool operator>(const SimpleNode& right) const { return cost > right.cost; }
};

inline double calcDistance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

inline double calcDistance(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
  return calcDistance(p1.position, p2.position);
}

inline double calcDistance(const geometry_msgs::PoseStamped& p1,
                           const geometry_msgs::PoseStamped& p2) {
  return calcDistance(p1.pose, p2.pose);
}

inline double modifyTheta(double theta) {
  if (theta < 0.0) return theta + 2.0 * M_PI;
  if (theta >= 2.0 * M_PI) return theta - 2.0 * M_PI;

  return theta;
}

inline geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose,
                                         const geometry_msgs::TransformStamped& transform) {
  geometry_msgs::Pose transformed_pose;
  tf2::doTransform(pose, transformed_pose, transform);

  return transformed_pose;
}

inline geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Pose pose,
                                                   geometry_msgs::Point point) {
  tf2::Transform tf_transform;
  tf2::convert(pose, tf_transform);

  geometry_msgs::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_transform.inverse());

  geometry_msgs::Point transformed_point;
  tf2::doTransform(point, transformed_point, transform);

  return transformed_point;
}

inline double calcDiffOfRadian(double a, double b) {
  double diff = std::fmod(std::fabs(a - b), 2.0 * M_PI);
  if (diff < M_PI)
    return diff;
  else
    return 2.0 * M_PI - diff;
}

inline geometry_msgs::Pose xytToPoseMsg(double x, double y, double theta) {
  geometry_msgs::Pose p;
  p.position.x = x;
  p.position.y = y;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, theta);
  tf2::convert(quat, p.orientation);

  return p;
}

#endif

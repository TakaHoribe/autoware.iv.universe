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

#include "astar_search/astar_search.h"

#include <vector>

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <astar_search/helper.h>

namespace {

double calcDistance2d(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return std::hypot(p2.x - p1.x, p2.y - p1.y);
}

double calcDistance2d(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
  return calcDistance2d(p1.position, p2.position);
}

geometry_msgs::Pose transformPose(const geometry_msgs::Pose& pose,
                                  const geometry_msgs::TransformStamped& transform) {
  geometry_msgs::Pose transformed_pose;
  tf2::doTransform(pose, transformed_pose, transform);

  return transformed_pose;
}

geometry_msgs::Point calcRelativeCoordinate(const geometry_msgs::Pose& pose,
                                            const geometry_msgs::Point& point) {
  tf2::Transform tf_transform;
  tf2::convert(pose, tf_transform);

  geometry_msgs::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_transform.inverse());

  geometry_msgs::Point transformed_point;
  tf2::doTransform(point, transformed_point, transform);

  return transformed_point;
}

geometry_msgs::Pose global2local(const nav_msgs::OccupancyGrid& costmap,
                                 const geometry_msgs::Pose& pose_global) {
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  geometry_msgs::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin.inverse());

  return transformPose(pose_global, transform);
}

geometry_msgs::Pose local2global(const nav_msgs::OccupancyGrid& costmap,
                                 const geometry_msgs::Pose& pose_local) {
  tf2::Transform tf_origin;
  tf2::convert(costmap.info.origin, tf_origin);

  geometry_msgs::TransformStamped transform;
  transform.transform = tf2::toMsg(tf_origin);

  return transformPose(pose_local, transform);
}

IndexXYT pose2index(const nav_msgs::OccupancyGrid& costmap, const geometry_msgs::Pose& pose_global,
                    const int theta_size) {
  const auto pose_local = global2local(costmap, pose_global);

  const int index_x = pose_local.position.x / costmap.info.resolution;
  const int index_y = pose_local.position.y / costmap.info.resolution;

  tf2::Quaternion quat;
  tf2::convert(pose_local.orientation, quat);
  double yaw = tf2::getYaw(quat);
  if (yaw < 0) {
    yaw += 2.0 * M_PI;
  }

  // Descretize angle
  const double one_angle_range = 2.0 * M_PI / theta_size;
  const int index_theta = static_cast<int>(yaw / one_angle_range) % theta_size;

  return {index_x, index_y, index_theta};
}

geometry_msgs::Pose index2pose(const nav_msgs::OccupancyGrid& costmap, const IndexXYT& index,
                               const int theta_size) {
  geometry_msgs::Pose pose_local;

  pose_local.position.x = index.x * costmap.info.resolution;
  pose_local.position.y = index.y * costmap.info.resolution;

  const double one_angle_range = 2.0 * M_PI / theta_size;
  const double yaw = index.theta * one_angle_range;
  tf2::Quaternion quat;
  quat.setRPY(0, 0, yaw);
  tf2::convert(quat, pose_local.orientation);

  return global2local(costmap, pose_local);
}

geometry_msgs::Pose node2pose(const AstarNode& node) {
  geometry_msgs::Pose pose;

  pose.position.x = node.x;
  pose.position.y = node.y;
  pose.position.z = 0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, node.theta);
  tf2::convert(quat, pose.orientation);

  return pose;
}

NodeUpdate rotate(const NodeUpdate& nu, const double theta) {
  NodeUpdate result = nu;
  result.shift_x = std::cos(theta) * nu.shift_x - std::sin(theta) * nu.shift_y;
  result.shift_y = std::sin(theta) * nu.shift_x + std::cos(theta) * nu.shift_y;
  return result;
}

NodeUpdate flip(const NodeUpdate& nu) {
  NodeUpdate result = nu;
  result.shift_y = -result.shift_y;
  result.shift_theta = -result.shift_theta;
  return result;
}

NodeUpdate reverse(const NodeUpdate& nu) {
  NodeUpdate result = nu;
  result.shift_x = -result.shift_x;
  result.shift_theta = -result.shift_theta;
  result.is_back = !result.is_back;
  return result;
}

AstarSearch::StateUpdateTable createStateUpdateTable(const double minimum_turning_radius,
                                                     const double theta_size, const bool use_back) {
  // Vehicle moving for each angle
  AstarSearch::StateUpdateTable state_update_table;
  state_update_table.resize(theta_size);

  const double dtheta = 2.0 * M_PI / theta_size;

  // Minimum moving distance with one state update
  // arc  = r * theta
  const auto& R = minimum_turning_radius;
  const double step = R * dtheta;

  // NodeUpdate actions
  const NodeUpdate forward_straight{step, 0.0, 0.0, step, false, false};
  const NodeUpdate forward_left{R * sin(dtheta), R * (1 - cos(dtheta)), dtheta, step, true, false};
  const NodeUpdate forward_right = flip(forward_left);
  const NodeUpdate backward_straight = reverse(forward_straight);
  const NodeUpdate backward_left = reverse(forward_left);
  const NodeUpdate backward_right = reverse(forward_right);

  for (int i = 0; i < theta_size; i++) {
    const double theta = dtheta * i;

    for (const auto& nu : {forward_straight, forward_left, forward_right}) {
      state_update_table[i].push_back(rotate(nu, theta));
    }

    if (use_back) {
      for (const auto& nu : {backward_straight, backward_left, backward_right}) {
        state_update_table[i].push_back(rotate(nu, theta));
      }
    }
  }

  return state_update_table;
}

}  // namespace

AstarSearch::AstarSearch(const AstarParam& astar_param) : astar_param_(astar_param) {
  state_update_table_ = createStateUpdateTable(astar_param_.minimum_turning_radius,
                                               astar_param_.theta_size, astar_param_.use_back);
}

void AstarSearch::initializeNodes(const nav_msgs::OccupancyGrid& costmap) {
  costmap_ = costmap;

  const auto height = costmap_.info.height;
  const auto width = costmap_.info.width;

  // size initialization
  nodes_.clear();
  nodes_.resize(height);
  for (int i = 0; i < height; i++) {
    nodes_[i].resize(width);
  }
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      nodes_[i][j].resize(astar_param_.theta_size);
    }
  }

  // cost initialization
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      // Index of subscribing OccupancyGrid message
      const int og_index = i * width + j;
      const int cost = costmap_.data[og_index];

      // hc is set to be 0 when reset()
      if (cost == 0) {
        continue;
      }

      // obstacle or unknown area
      if (cost < 0 || astar_param_.obstacle_threshold <= cost) {
        nodes_[i][j][0].status = NodeStatus::Obstacle;
      }
    }
  }
}

bool AstarSearch::makePlan(const geometry_msgs::Pose& start_pose,
                           const geometry_msgs::Pose& goal_pose) {
  start_pose_ = start_pose;
  goal_pose_ = goal_pose;

  if (!setStartNode()) {
    return false;
  }

  if (!setGoalNode()) {
    return false;
  }

  return search();
}

bool AstarSearch::setStartNode() {
  // Get index of start pose
  const auto index = pose2index(costmap_, start_pose_, astar_param_.theta_size);

  if (isOutOfRange(index.x, index.y)) {
    return false;
  }

  if (detectCollision(index)) {
    return false;
  }

  // Set start node
  AstarNode& start_node = nodes_[index.y][index.x][index.theta];
  start_node.index = index;
  start_node.x = start_pose_.position.x;
  start_node.y = start_pose_.position.y;
  start_node.theta = 2.0 * M_PI / astar_param_.theta_size * index.theta;
  start_node.move_distance = 0;
  start_node.is_back = false;
  start_node.status = NodeStatus::Open;
  start_node.parent = nullptr;

  start_node.gc = 0;
  start_node.hc = calcDistance2d(start_pose_, goal_pose_) * astar_param_.distance_heuristic_weight;
  start_node.cost = start_node.gc + start_node.hc;

  // Push start node to openlist
  openlist_.push(start_node);

  return true;
}

bool AstarSearch::setGoalNode() {
  goal_yaw_ = normalizeRadian(tf2::getYaw(goal_pose_.orientation), 0, 2 * M_PI);

  // Get index of goal pose
  const auto index = pose2index(costmap_, goal_pose_, astar_param_.theta_size);

  if (isOutOfRange(index.x, index.y)) {
    return false;
  }

  if (detectCollision(index)) {
    return false;
  }

  return true;
}

bool AstarSearch::search() {
  const ros::WallTime begin = ros::WallTime::now();

  // Start A* search
  while (!openlist_.empty()) {
    // Check time and terminate if the search reaches the time limit
    const ros::WallTime now = ros::WallTime::now();
    const double msec = (now - begin).toSec() * 1000.0;
    if (msec > astar_param_.time_limit) {
      return false;
    }

    // Pop minimum cost node from openlist
    const auto top_sn = openlist_.top();
    openlist_.pop();

    // Expand nodes from this node
    AstarNode* current_an = &nodes_[top_sn.index.y][top_sn.index.x][top_sn.index.theta];
    current_an->status = NodeStatus::Closed;

    if (isGoal(current_an->x, current_an->y, current_an->theta)) {
      setPath(top_sn);
      return true;
    }

    // Expand nodes
    for (const auto& state : state_update_table_[top_sn.index.theta]) {
      // Next state
      const double next_x = current_an->x + state.shift_x;
      const double next_y = current_an->y + state.shift_y;
      const double next_theta = normalizeRadian(current_an->theta + state.shift_theta, 0, 2 * M_PI);
      const double move_distance = current_an->move_distance + state.step;

      const auto is_turning_point = state.is_back != current_an->is_back;
      const double move_cost =
          is_turning_point ? astar_param_.reverse_weight * state.step : state.step;

      // Calculate index of the next state
      geometry_msgs::Pose next_pose;
      next_pose.position.x = next_x;
      next_pose.position.y = next_y;

      tf2::Quaternion quat;
      quat.setRPY(0, 0, next_theta);
      tf2::convert(quat, next_pose.orientation);

      const auto next_index = pose2index(costmap_, next_pose, astar_param_.theta_size);

      // Check if the index is valid
      if (isOutOfRange(next_index.x, next_index.y)) {
        continue;
      }

      if (detectCollision(next_index)) {
        continue;
      }

      AstarNode* next_an = &nodes_[next_index.y][next_index.x][next_index.theta];
      next_an->index = next_index;
      const double next_gc = current_an->gc + move_cost;
      const double next_hc =
          calcDistance2d(next_pose, goal_pose_) * astar_param_.distance_heuristic_weight;

      if (next_an->status == NodeStatus::None) {
        next_an->status = NodeStatus::Open;
        next_an->x = next_x;
        next_an->y = next_y;
        next_an->theta = next_theta;
        next_an->gc = next_gc;
        next_an->hc = next_hc;
        next_an->move_distance = move_distance;
        next_an->is_back = state.is_back;
        next_an->parent = current_an;
        next_an->cost = next_an->gc + next_an->hc;
        openlist_.push(*next_an);
        continue;
      }

      if (next_an->status == NodeStatus::Open || next_an->status == NodeStatus::Closed) {
        if (next_gc < next_an->gc) {
          next_an->status = NodeStatus::Open;
          next_an->x = next_x;
          next_an->y = next_y;
          next_an->theta = next_theta;
          next_an->gc = next_gc;
          next_an->hc = next_hc;  // already calculated ?
          next_an->move_distance = move_distance;
          next_an->is_back = state.is_back;
          next_an->parent = current_an;
          next_an->cost = next_an->gc + next_an->hc;
          openlist_.push(*next_an);
          continue;
        }
      }
    }  // state update
  }

  // Failed to find path
  return false;
}

void AstarSearch::setPath(const AstarNode& goal_node) {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = costmap_.header.frame_id;

  path_.header = header;
  path_.poses.clear();

  waypoints_.header = header;
  waypoints_.waypoints.clear();

  // From the goal node to the start node
  const AstarNode* node = &goal_node;

  while (node != nullptr) {
    geometry_msgs::PoseStamped pose;
    pose.header = header;
    pose.pose = node2pose(*node);

    path_.poses.push_back(pose);

    // Set AstarWaypoints
    AstarWaypoint aw;
    aw.pose = pose;
    aw.is_back = node->is_back;
    waypoints_.waypoints.push_back(aw);

    // To the next node
    node = node->parent;
  }

  // Reverse the vector to be start to goal order
  std::reverse(path_.poses.begin(), path_.poses.end());
  std::reverse(waypoints_.waypoints.begin(), waypoints_.waypoints.end());

  // Update first point direction
  if (waypoints_.waypoints.size() > 1) {
    waypoints_.waypoints.at(0).is_back = waypoints_.waypoints.at(1).is_back;
  }
}

bool AstarSearch::detectCollision(const IndexXYT& index) {
  // Define the robot as rectangle
  const double back = -1.0 * astar_param_.robot_base2back;
  const double front = astar_param_.robot_length - astar_param_.robot_base2back;

  const double right = -1.0 * astar_param_.robot_width / 2.0;
  const double left = astar_param_.robot_width / 2.0;

  const double resolution = costmap_.info.resolution;

  // Coordinate of base_link in OccupancyGrid frame
  const double one_angle_range = 2.0 * M_PI / astar_param_.theta_size;
  const double base_x = index.x * resolution;
  const double base_y = index.y * resolution;
  const double base_theta = index.theta * one_angle_range;

  // Calculate cos and sin in advance
  const double cos_theta = std::cos(base_theta);
  const double sin_theta = std::sin(base_theta);

  // Convert each point to index and check if the node is Obstacle
  for (double x = back; x <= front; x += resolution) {
    for (double y = right; y <= left; y += resolution) {
      // 2D point rotation
      const int index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
      const int index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;

      if (isOutOfRange(index_x, index_y)) {
        return true;
      }

      if (isObs(index_x, index_y)) {
        return true;
      }
    }
  }

  return false;
}

bool AstarSearch::hasObstacleOnTrajectory(const geometry_msgs::PoseArray& trajectory) {
  for (const auto& p : trajectory.poses) {
    const auto index = pose2index(costmap_, p, astar_param_.theta_size);

    if (isOutOfRange(index.x, index.y)) {
      continue;
    }

    if (isObs(index.x, index.y)) {
      return true;
    }
  }

  return false;
}

bool AstarSearch::isOutOfRange(const int index_x, const int index_y) {
  if (index_x < 0 || index_x >= static_cast<int>(costmap_.info.width)) return true;
  if (index_y < 0 || index_y >= static_cast<int>(costmap_.info.height)) return true;
  return false;
}

bool AstarSearch::isObs(const int index_x, const int index_y) {
  return (nodes_[index_y][index_x][0].status == NodeStatus::Obstacle);
}

// Check if the next state is the goal
// Check lateral offset, longitudinal offset and angle
bool AstarSearch::isGoal(const double x, const double y, const double theta) {
  // To reduce computation time, we use square value for distance
  const double lateral_goal_range =
      astar_param_.lateral_goal_range / 2.0;  // [meter], divide by 2 means we check left and right
  const double longitudinal_goal_range =
      astar_param_.longitudinal_goal_range / 2.0;  // [meter], check only behind of the goal
  const double goal_angle =
      M_PI * (astar_param_.angle_goal_range / 2.0) / 180.0;  // degrees -> radian

  // Calculate the node coordinate seen from the goal point
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = 0;

  geometry_msgs::Point relative_node_point = calcRelativeCoordinate(goal_pose_, p);

  // Check Pose of goal
  if (relative_node_point.x < 0 &&  // shoud be behind of goal
      std::fabs(relative_node_point.x) < longitudinal_goal_range &&
      std::fabs(relative_node_point.y) < lateral_goal_range) {
    // Check the orientation of goal
    if (std::abs(normalizeRadian(goal_yaw_ - theta)) < goal_angle) {
      return true;
    }
  }

  return false;
}

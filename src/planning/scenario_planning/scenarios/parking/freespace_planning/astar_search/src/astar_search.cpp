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

IndexXYT pose2index(const nav_msgs::OccupancyGrid& costmap, const geometry_msgs::Pose& pose_local,
                    const int theta_size) {
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

  return pose_local;
}

geometry_msgs::Pose node2pose(const AstarNode& node) {
  geometry_msgs::Pose pose_local;

  pose_local.position.x = node.x;
  pose_local.position.y = node.y;
  pose_local.position.z = 0;

  tf2::Quaternion quat;
  quat.setRPY(0, 0, node.theta);
  tf2::convert(quat, pose_local.orientation);

  return pose_local;
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
  start_pose_ = global2local(costmap_, start_pose);
  goal_pose_ = global2local(costmap_, goal_pose);

  if (!setStartNode()) {
    return false;
  }

  if (!setGoalNode()) {
    return false;
  }

  return search();
}

bool AstarSearch::setStartNode() {
  const auto index = pose2index(costmap_, start_pose_, astar_param_.theta_size);

  if (isOutOfRange(index)) {
    return false;
  }

  if (detectCollision(index)) {
    return false;
  }

  // Set start node
  AstarNode* start_node = getNodeRef(index);
  start_node->index = index;
  start_node->x = start_pose_.position.x;
  start_node->y = start_pose_.position.y;
  start_node->theta = 2.0 * M_PI / astar_param_.theta_size * index.theta;
  start_node->gc = 0;
  start_node->hc = calcDistance2d(start_pose_, goal_pose_) * astar_param_.distance_heuristic_weight;
  start_node->cost = start_node->gc + start_node->hc;
  start_node->move_distance = 0;
  start_node->is_back = false;
  start_node->status = NodeStatus::Open;
  start_node->parent = nullptr;

  // Push start node to openlist
  openlist_.push(start_node);

  return true;
}

bool AstarSearch::setGoalNode() {
  const auto index = pose2index(costmap_, goal_pose_, astar_param_.theta_size);

  if (isOutOfRange(index)) {
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
    const auto& top_node = *openlist_.top();
    openlist_.pop();

    // Expand nodes from this node
    AstarNode* current_an = getNodeRef(top_node.index);
    current_an->status = NodeStatus::Closed;

    if (isGoal(current_an->x, current_an->y, current_an->theta)) {
      setPath(top_node);
      return true;
    }

    // Expand nodes
    for (const auto& state : state_update_table_[top_node.index.theta]) {
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
      if (isOutOfRange(next_index)) {
        continue;
      }

      if (detectCollision(next_index)) {
        continue;
      }

      AstarNode* next_node = getNodeRef(next_index);
      next_node->index = next_index;
      const double next_gc = current_an->gc + move_cost;
      const double next_hc =
          calcDistance2d(next_pose, goal_pose_) * astar_param_.distance_heuristic_weight;

      if (next_node->status == NodeStatus::None) {
        next_node->status = NodeStatus::Open;
        next_node->x = next_x;
        next_node->y = next_y;
        next_node->theta = next_theta;
        next_node->gc = next_gc;
        next_node->hc = next_hc;
        next_node->cost = next_node->gc + next_node->hc;
        next_node->move_distance = move_distance;
        next_node->is_back = state.is_back;
        next_node->parent = current_an;
        openlist_.push(next_node);
        continue;
      }

      if (next_node->status == NodeStatus::Open || next_node->status == NodeStatus::Closed) {
        if (next_gc < next_node->gc) {
          next_node->status = NodeStatus::Open;
          next_node->x = next_x;
          next_node->y = next_y;
          next_node->theta = next_theta;
          next_node->gc = next_gc;
          next_node->hc = next_hc;  // already calculated ?
          next_node->cost = next_node->gc + next_node->hc;
          next_node->move_distance = move_distance;
          next_node->is_back = state.is_back;
          next_node->parent = current_an;
          openlist_.push(next_node);
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
    pose.pose = local2global(costmap_, node2pose(*node));

    // Path
    path_.poses.push_back(pose);

    // AstarWaypoints
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

bool AstarSearch::detectCollision(const IndexXYT& base_index) {
  const RobotShape& robot_shape = astar_param_.robot_shape;

  // Define the robot as rectangle
  const double back = -1.0 * robot_shape.base2back;
  const double front = robot_shape.length - robot_shape.base2back;

  const double right = -1.0 * robot_shape.width / 2.0;
  const double left = robot_shape.width / 2.0;

  const auto base_pose = index2pose(costmap_, base_index, astar_param_.theta_size);
  const auto base_theta = tf2::getYaw(base_pose.orientation);

  // Convert each point to index and check if the node is Obstacle
  for (double x = back; x <= front; x += costmap_.info.resolution) {
    for (double y = right; y <= left; y += costmap_.info.resolution) {
      // Calculate offset in rotated frame
      const double offset_x = std::cos(base_theta) * x - std::sin(base_theta) * y;
      const double offset_y = std::sin(base_theta) * x + std::cos(base_theta) * y;

      geometry_msgs::Pose pose_local;
      pose_local.position.x = base_pose.position.x + offset_x;
      pose_local.position.y = base_pose.position.y + offset_y;

      const auto index = pose2index(costmap_, pose_local, astar_param_.theta_size);

      if (isOutOfRange(index)) {
        return true;
      }

      if (isObs(index)) {
        return true;
      }
    }
  }

  return false;
}

bool AstarSearch::hasObstacleOnTrajectory(const geometry_msgs::PoseArray& trajectory) {
  for (const auto& pose : trajectory.poses) {
    const auto pose_local = global2local(costmap_, pose);
    const auto index = pose2index(costmap_, pose_local, astar_param_.theta_size);

    if (detectCollision(index)) {
      return true;
    }
  }

  return false;
}

bool AstarSearch::isOutOfRange(const IndexXYT& index) {
  if (index.x < 0 || static_cast<int>(costmap_.info.width) <= index.x) return true;
  if (index.y < 0 || static_cast<int>(costmap_.info.height) <= index.y) return true;
  return false;
}

bool AstarSearch::isObs(const IndexXYT& index) {
  return (nodes_[index.y][index.x][0].status == NodeStatus::Obstacle);
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

  // Check conditions
  if (astar_param_.only_behind_solutions && relative_node_point.x > 0) {
    return false;
  }

  if (std::fabs(relative_node_point.x) > longitudinal_goal_range ||
      std::fabs(relative_node_point.y) > lateral_goal_range) {
    return false;
  }

  if (std::abs(normalizeRadian(tf2::getYaw(goal_pose_.orientation) - theta)) > goal_angle) {
    return false;
  }

  return true;
}

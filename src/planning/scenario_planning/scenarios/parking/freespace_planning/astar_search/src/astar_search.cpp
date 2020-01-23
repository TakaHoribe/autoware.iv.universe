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

#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace {

IndexXYT pose2index(const nav_msgs::OccupancyGrid& costmap, const geometry_msgs::Pose& pose,
                    const int theta_size) {
  tf2::Transform orig_tf;
  tf2::convert(costmap.info.origin, orig_tf);

  geometry_msgs::TransformStamped transform;
  transform.transform = tf2::toMsg(orig_tf.inverse());
  geometry_msgs::Pose pose2d = transformPose(pose, transform);

  const int index_x = pose2d.position.x / costmap.info.resolution;
  const int index_y = pose2d.position.y / costmap.info.resolution;

  tf2::Quaternion quat;
  tf2::convert(pose2d.orientation, quat);
  double yaw = tf2::getYaw(quat);
  if (yaw < 0) {
    yaw += 2.0 * M_PI;
  }

  // Descretize angle
  static const double one_angle_range = 2.0 * M_PI / theta_size;
  const int index_theta = static_cast<int>(yaw / one_angle_range) % theta_size;

  return {index_x, index_y, index_theta};
}

IndexXY point2index(const nav_msgs::OccupancyGrid& costmap, const geometry_msgs::Point& point) {
  geometry_msgs::Pose pose;
  pose.position = point;
  const auto index = pose2index(costmap, pose, 1);
  return {index.x, index.y};
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

AstarSearch::StateUpdateTable createStateUpdateTable(const double minimum_turning_radius,
                                                     const double theta_size, const bool use_back) {
  // Vehicle moving for each angle
  AstarSearch::StateUpdateTable state_update_table;
  state_update_table.resize(theta_size);
  const double dtheta = 2.0 * M_PI / theta_size;

  // Minimum moving distance with one state update
  // arc  = r * theta
  const double step = minimum_turning_radius * dtheta;

  for (int i = 0; i < theta_size; i++) {
    const double theta = dtheta * i;

    // Calculate right and left circle
    // Robot moves along these circles
    const double right_circle_center_x = minimum_turning_radius * std::sin(theta);
    const double right_circle_center_y = minimum_turning_radius * -std::cos(theta);
    const double left_circle_center_x = -right_circle_center_x;
    const double left_circle_center_y = -right_circle_center_y;

    // Calculate x and y shift to next state
    NodeUpdate nu;

    // forward straight
    nu.shift_x = step * std::cos(theta);
    nu.shift_y = step * std::sin(theta);
    nu.rotation = 0.0;
    nu.index_theta = 0;
    nu.step = step;
    nu.is_curve = false;
    nu.is_back = false;
    state_update_table[i].emplace_back(nu);

    // forward right
    nu.shift_x = right_circle_center_x + minimum_turning_radius * std::cos(M_PI_2 + theta - dtheta);
    nu.shift_y = right_circle_center_y + minimum_turning_radius * std::sin(M_PI_2 + theta - dtheta);
    nu.rotation = -dtheta;
    nu.index_theta = -1;
    nu.step = step;
    nu.is_curve = true;
    nu.is_back = false;
    state_update_table[i].emplace_back(nu);

    // forward left
    nu.shift_x = left_circle_center_x + minimum_turning_radius * std::cos(-M_PI_2 + theta + dtheta);
    nu.shift_y = left_circle_center_y + minimum_turning_radius * std::sin(-M_PI_2 + theta + dtheta);
    nu.rotation = dtheta;
    nu.index_theta = 1;
    nu.step = step;
    nu.is_curve = true;
    nu.is_back = false;
    state_update_table[i].emplace_back(nu);

    if (use_back) {
      // backward straight
      nu.shift_x = step * std::cos(theta) * -1.0;
      nu.shift_y = step * std::sin(theta) * -1.0;
      nu.rotation = 0;
      nu.index_theta = 0;
      nu.step = step;
      nu.is_curve = false;
      nu.is_back = true;
      state_update_table[i].emplace_back(nu);

      // backward right
      nu.shift_x =
          right_circle_center_x + minimum_turning_radius * std::cos(M_PI_2 + theta + dtheta);
      nu.shift_y =
          right_circle_center_y + minimum_turning_radius * std::sin(M_PI_2 + theta + dtheta);
      nu.rotation = dtheta;
      nu.index_theta = 1;
      nu.step = step;
      nu.is_curve = true;
      nu.is_back = true;
      state_update_table[i].emplace_back(nu);

      // backward left
      nu.shift_x =
          left_circle_center_x + minimum_turning_radius * std::cos(-1.0 * M_PI_2 + theta - dtheta);
      nu.shift_y =
          left_circle_center_y + minimum_turning_radius * std::sin(-1.0 * M_PI_2 + theta - dtheta);
      nu.rotation = dtheta * -1.0;
      nu.index_theta = -1;
      nu.step = step;
      nu.is_curve = true;
      nu.is_back = true;
      state_update_table[i].emplace_back(nu);
    }
  }

  return state_update_table;
}

}  // namespace

AstarSearch::AstarSearch() : nh_(""), private_nh_("~") {
  // base configs
  private_nh_.param<bool>("use_back", use_back_, true);
  private_nh_.param<bool>("use_potential_heuristic", use_potential_heuristic_, true);
  private_nh_.param<bool>("use_wavefront_heuristic", use_wavefront_heuristic_, false);
  private_nh_.param<double>("time_limit", time_limit_, 5000.0);

  // robot configs
  private_nh_.param<double>("robot_length", robot_length_, 4.5);
  private_nh_.param<double>("robot_width", robot_width_, 1.75);
  private_nh_.param<double>("robot_base2back", robot_base2back_, 1.0);
  private_nh_.param<double>("minimum_turning_radius", minimum_turning_radius_, 6.0);

  // search configs
  private_nh_.param<int>("theta_size", theta_size_, 48);
  private_nh_.param<double>("angle_goal_range", angle_goal_range_, 6.0);
  private_nh_.param<double>("curve_weight", curve_weight_, 1.2);
  private_nh_.param<double>("reverse_weight", reverse_weight_, 2.00);
  private_nh_.param<double>("lateral_goal_range", lateral_goal_range_, 0.5);
  private_nh_.param<double>("longitudinal_goal_range", longitudinal_goal_range_, 2.0);

  // costmap configs
  private_nh_.param<int>("obstacle_threshold", obstacle_threshold_, 100);
  private_nh_.param<double>("potential_weight", potential_weight_, 10.0);
  private_nh_.param<double>("distance_heuristic_weight", distance_heuristic_weight_, 1.0);

  state_update_table_ = createStateUpdateTable(minimum_turning_radius_, theta_size_, use_back_);
}

void AstarSearch::initializeNodes(const nav_msgs::OccupancyGrid& costmap) {
  costmap_ = costmap;

  const auto height = costmap_.info.height;
  const auto width = costmap_.info.width;

  // size initialization
  nodes_.resize(height);
  for (int i = 0; i < height; i++) {
    nodes_[i].resize(width);
  }
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      nodes_[i][j].resize(theta_size_);
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
      if (cost < 0 || obstacle_threshold_ <= cost) {
        nodes_[i][j][0].status = Status::OBS;
      }

      // the cost more than threshold is regarded almost same as an obstacle
      // because of its very high cost
      if (use_potential_heuristic_) {
        nodes_[i][j][0].hc = cost * potential_weight_;
      }
    }
  }
}

bool AstarSearch::makePlan(const geometry_msgs::Pose& start_pose,
                           const geometry_msgs::Pose& goal_pose) {
  if (!setStartNode(start_pose)) {
    return false;
  }

  if (!setGoalNode(goal_pose)) {
    return false;
  }

  return search();
}

bool AstarSearch::setStartNode(const geometry_msgs::Pose& start_pose) {
  start_pose_local_.pose = start_pose;

  // Get index of start pose
  const auto index = pose2index(costmap_, start_pose_local_.pose, theta_size_);
  SimpleNode start_sn{index, 0};

  if (isOutOfRange(start_sn.index.x, start_sn.index.y)) {
    return false;
  }

  if (detectCollision(start_sn)) {
    return false;
  }

  // Set start node
  AstarNode& start_node = nodes_[index.y][index.x][index.theta];
  start_node.x = start_pose_local_.pose.position.x;
  start_node.y = start_pose_local_.pose.position.y;
  start_node.theta = 2.0 * M_PI / theta_size_ * index.theta;
  start_node.gc = 0;
  start_node.move_distance = 0;
  start_node.is_back = false;
  start_node.status = Status::OPEN;
  start_node.parent = nullptr;

  // set euclidean distance heuristic cost
  if (!use_wavefront_heuristic_ && !use_potential_heuristic_) {
    start_node.hc = calcDistance(start_pose_local_, goal_pose_local_) * distance_heuristic_weight_;
  } else if (use_potential_heuristic_) {
    start_node.gc += start_node.hc;
    start_node.hc += calcDistance(start_pose_local_, goal_pose_local_) * distance_heuristic_weight_;
  }

  // Push start node to openlist
  start_sn.cost = start_node.gc + start_node.hc;
  openlist_.push(start_sn);

  return true;
}

bool AstarSearch::setGoalNode(const geometry_msgs::Pose& goal_pose) {
  goal_pose_local_.pose = goal_pose;
  goal_yaw_ = modifyTheta(tf2::getYaw(goal_pose_local_.pose.orientation));

  // Get index of goal pose
  const auto index = pose2index(costmap_, goal_pose_local_.pose, theta_size_);
  const SimpleNode goal_sn{index, 0};

  if (isOutOfRange(goal_sn.index.x, goal_sn.index.y)) {
    return false;
  }

  if (detectCollision(goal_sn)) {
    return false;
  }

  if (use_wavefront_heuristic_) {
    const bool is_reachable = calcWaveFrontHeuristic(goal_sn);
    if (!is_reachable) {
      return false;
    }
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
    if (msec > time_limit_) {
      return false;
    }

    // Pop minimum cost node from openlist
    const SimpleNode top_sn = openlist_.top();
    openlist_.pop();

    // Expand nodes from this node
    AstarNode* current_an = &nodes_[top_sn.index.y][top_sn.index.x][top_sn.index.theta];
    current_an->status = Status::CLOSED;

    if (isGoal(current_an->x, current_an->y, current_an->theta)) {
      setPath(top_sn);
      return true;
    }

    // Expand nodes
    for (const auto& state : state_update_table_[top_sn.index.theta]) {
      // Next state
      const double next_x = current_an->x + state.shift_x;
      const double next_y = current_an->y + state.shift_y;
      const double next_theta = modifyTheta(current_an->theta + state.rotation);
      const double move_distance = current_an->move_distance + state.step;

      const auto is_turning_point = state.is_back != current_an->is_back;
      const double move_cost = is_turning_point ? reverse_weight_ * state.step : state.step;

      // Calculate index of the next state
      geometry_msgs::Point next_pos;
      next_pos.x = next_x;
      next_pos.y = next_y;

      const auto index = point2index(costmap_, next_pos);

      SimpleNode next_sn{IndexXYT{index.x, index.y, top_sn.index.theta + state.index_theta}, 0};

      // Avoid invalid index
      next_sn.index.theta = (next_sn.index.theta + theta_size_) % theta_size_;

      // Check if the index is valid
      if (isOutOfRange(next_sn.index.x, next_sn.index.y)) {
        continue;
      }

      if (detectCollision(next_sn)) {
        continue;
      }

      AstarNode* next_an = &nodes_[next_sn.index.y][next_sn.index.x][next_sn.index.theta];
      double next_gc = current_an->gc + move_cost;
      // wavefront or distance transform heuristic
      double next_hc = nodes_[next_sn.index.y][next_sn.index.x][0].hc;

      // increase the cost with euclidean distance
      if (use_potential_heuristic_) {
        next_gc += nodes_[next_sn.index.y][next_sn.index.x][0].hc;
        next_hc +=
            calcDistance(next_pos, goal_pose_local_.pose.position) * distance_heuristic_weight_;
      }

      // increase the cost with euclidean distance
      if (!use_wavefront_heuristic_ && !use_potential_heuristic_) {
        next_hc =
            calcDistance(next_pos, goal_pose_local_.pose.position) * distance_heuristic_weight_;
      }

      if (next_an->status == Status::NONE) {
        next_an->status = Status::OPEN;
        next_an->x = next_x;
        next_an->y = next_y;
        next_an->theta = next_theta;
        next_an->gc = next_gc;
        next_an->hc = next_hc;
        next_an->move_distance = move_distance;
        next_an->is_back = state.is_back;
        next_an->parent = current_an;
        next_sn.cost = next_an->gc + next_an->hc;
        openlist_.push(next_sn);
        continue;
      }

      if (next_an->status == Status::OPEN || next_an->status == Status::CLOSED) {
        if (next_gc < next_an->gc) {
          next_an->status = Status::OPEN;
          next_an->x = next_x;
          next_an->y = next_y;
          next_an->theta = next_theta;
          next_an->gc = next_gc;
          next_an->hc = next_hc;  // already calculated ?
          next_an->move_distance = move_distance;
          next_an->is_back = state.is_back;
          next_an->parent = current_an;
          next_sn.cost = next_an->gc + next_an->hc;
          openlist_.push(next_sn);
          continue;
        }
      }
    }  // state update
  }

  // Failed to find path
  return false;
}

void AstarSearch::setPath(const SimpleNode& goal) {
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  header.frame_id = costmap_.header.frame_id;

  path_.header = header;
  path_.poses.clear();

  waypoints_.header = header;
  waypoints_.waypoints.clear();

  // From the goal node to the start node
  AstarNode* node = &nodes_[goal.index.y][goal.index.x][goal.index.theta];

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

bool AstarSearch::detectCollision(const SimpleNode& sn) {
  // Define the robot as rectangle
  static const double left = -1.0 * robot_base2back_;
  static const double right = robot_length_ - robot_base2back_;
  static const double top = robot_width_ / 2.0;
  static const double bottom = -1.0 * robot_width_ / 2.0;
  static const double resolution = costmap_.info.resolution;

  // Coordinate of base_link in OccupancyGrid frame
  static const double one_angle_range = 2.0 * M_PI / theta_size_;
  const double base_x = sn.index.x * resolution;
  const double base_y = sn.index.y * resolution;
  const double base_theta = sn.index.theta * one_angle_range;

  // Calculate cos and sin in advance
  const double cos_theta = std::cos(base_theta);
  const double sin_theta = std::sin(base_theta);

  // Convert each point to index and check if the node is Obstacle
  for (double x = left; x < right; x += resolution) {
    for (double y = top; y > bottom; y -= resolution) {
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

bool AstarSearch::calcWaveFrontHeuristic(const SimpleNode& sn) {
  // Set start point for wavefront search
  // This is goal for Astar search
  nodes_[sn.index.y][sn.index.x][0].hc = 0;
  WaveFrontNode wf_node{IndexXY{sn.index.x, sn.index.y}, 1e-10};

  std::queue<WaveFrontNode> qu;
  qu.push(wf_node);

  // State update table for wavefront search
  // Nodes are expanded for each neighborhood cells (moore neighborhood)
  const double resolution = costmap_.info.resolution;
  static const std::vector<WaveFrontNode> updates = {
      WaveFrontNode{IndexXY{0, 1}, resolution},
      WaveFrontNode{IndexXY{-1, 0}, resolution},
      WaveFrontNode{IndexXY{1, 0}, resolution},
      WaveFrontNode{IndexXY{0, -1}, resolution},
      WaveFrontNode{IndexXY{-1, 1}, std::hypot(resolution, resolution)},
      WaveFrontNode{IndexXY{1, 1}, std::hypot(resolution, resolution)},
      WaveFrontNode{IndexXY{-1, -1}, std::hypot(resolution, resolution)},
      WaveFrontNode{IndexXY{1, -1}, std::hypot(resolution, resolution)},
  };

  const auto start_index = pose2index(costmap_, start_pose_local_.pose, theta_size_);

  // Whether the robot can reach goal
  bool is_reachable = false;

  // Start wavefront search
  while (!qu.empty()) {
    WaveFrontNode ref = qu.front();
    qu.pop();

    WaveFrontNode next;
    for (const auto& u : updates) {
      next.index.x = ref.index.x + u.index.x;
      next.index.y = ref.index.y + u.index.y;

      // out of range OR already visited OR obstacle node
      if (isOutOfRange(next.index.x, next.index.y)) {
        continue;
      }

      if (isObs(next.index.x, next.index.y)) {
        continue;
      }

      if (nodes_[next.index.y][next.index.x][0].hc > 0) {
        continue;
      }

      // Take the size of robot into account
      if (detectCollisionWaveFront(next)) {
        continue;
      }

      // Check if we can reach from start to goal
      if (next.index.x == start_index.x && next.index.y == start_index.y) {
        is_reachable = true;
      }

      // Set wavefront heuristic cost
      next.hc = ref.hc + u.hc;
      nodes_[next.index.y][next.index.x][0].hc = next.hc;

      qu.push(next);
    }
  }

  // End of search
  return is_reachable;
}

// Simple collidion detection for wavefront search
bool AstarSearch::detectCollisionWaveFront(const WaveFrontNode& ref) {
  // Define the robot as square
  static const double half = robot_width_ / 2;
  const double robot_x = ref.index.x * costmap_.info.resolution;
  const double robot_y = ref.index.y * costmap_.info.resolution;

  for (double y = half; y > -1.0 * half; y -= costmap_.info.resolution) {
    for (double x = -1.0 * half; x < half; x += costmap_.info.resolution) {
      const int index_x = (robot_x + x) / costmap_.info.resolution;
      const int index_y = (robot_y + y) / costmap_.info.resolution;

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

bool AstarSearch::hasObstacleOnPath() {
  for (const auto& p : waypoints_.waypoints) {
    const auto index = pose2index(costmap_, p.pose.pose, theta_size_);
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
  return (nodes_[index_y][index_x][0].status == Status::OBS);
}

// Check if the next state is the goal
// Check lateral offset, longitudinal offset and angle
bool AstarSearch::isGoal(const double x, const double y, const double theta) {
  // To reduce computation time, we use square value for distance
  static const double lateral_goal_range =
      lateral_goal_range_ / 2.0;  // [meter], divide by 2 means we check left and right
  static const double longitudinal_goal_range =
      longitudinal_goal_range_ / 2.0;  // [meter], check only behind of the goal
  static const double goal_angle = M_PI * (angle_goal_range_ / 2.0) / 180.0;  // degrees -> radian

  // Calculate the node coordinate seen from the goal point
  geometry_msgs::Point p;
  p.x = x;
  p.y = y;
  p.z = 0;

  geometry_msgs::Point relative_node_point = calcRelativeCoordinate(goal_pose_local_.pose, p);

  // Check Pose of goal
  if (relative_node_point.x < 0 &&  // shoud be behind of goal
      std::fabs(relative_node_point.x) < longitudinal_goal_range &&
      std::fabs(relative_node_point.y) < lateral_goal_range) {
    // Check the orientation of goal
    if (calcDiffOfRadian(goal_yaw_, theta) < goal_angle) {
      return true;
    }
  }

  return false;
}

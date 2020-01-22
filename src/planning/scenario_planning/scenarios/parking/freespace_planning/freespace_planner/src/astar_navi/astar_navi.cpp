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

#include "freespace_planner/astar_navi.h"

namespace {

bool isActive(const autoware_planning_msgs::Scenario& scenario) {
  const auto& s = scenario.activating_scenarios;
  if (std::find(std::begin(s), std::end(s), autoware_planning_msgs::Scenario::Parking) !=
      std::end(s)) {
    return true;
  }

  return false;
}

std::vector<size_t> getReversingIndices(const autoware_planning_msgs::Trajectory& trajectory) {
  std::vector<size_t> indices;

  for (size_t i = 0; i < trajectory.points.size() - 1; ++i) {
    if (trajectory.points.at(i).twist.linear.x * trajectory.points.at(i + 1).twist.linear.x < 0) {
      indices.push_back(i);
    }
  }

  return indices;
}

size_t getNextTargetIndex(const size_t trajectory_size,
                          const std::vector<size_t>& reversing_indices,
                          const size_t current_target_index) {
  if (!reversing_indices.empty()) {
    for (const auto reversing_index : reversing_indices) {
      if (reversing_index > current_target_index) {
        return reversing_index;
      }
    }
  }

  return trajectory_size - 1;
}

autoware_planning_msgs::Trajectory getPartialTrajectory(
    const autoware_planning_msgs::Trajectory& trajectory, const size_t start_index,
    const size_t end_index) {
  autoware_planning_msgs::Trajectory partial_trajectory;
  partial_trajectory.header = trajectory.header;
  partial_trajectory.header.stamp = ros::Time::now();

  partial_trajectory.points.reserve(trajectory.points.size());
  for (size_t i = start_index; i <= end_index; ++i) {
    partial_trajectory.points.push_back(trajectory.points.at(i));
  }

  // Modify velocity at start/end point
  if (partial_trajectory.points.size() >= 2) {
    partial_trajectory.points.front().twist.linear.x =
        partial_trajectory.points.at(1).twist.linear.x;
  }
  partial_trajectory.points.back().twist.linear.x = 0;

  return partial_trajectory;
}

double calculateDistance2d(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double calculateDistance2d(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
  return calculateDistance2d(p1.position, p2.position);
}

geometry_msgs::PoseStamped tf2pose(const geometry_msgs::TransformStamped& tf) {
  geometry_msgs::PoseStamped pose;

  pose.header = tf.header;
  pose.pose.orientation = tf.transform.rotation;
  pose.pose.position.x = tf.transform.translation.x;
  pose.pose.position.y = tf.transform.translation.y;
  pose.pose.position.z = tf.transform.translation.z;

  return pose;
}

geometry_msgs::TransformStamped getTransform(const tf2_ros::Buffer& tf_buffer,
                                             const std::string& from, const std::string& to) {
  geometry_msgs::TransformStamped tf;
  try {
    tf = tf_buffer.lookupTransform(from, to, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("%s", ex.what());
  }
  return tf;
}

autoware_planning_msgs::Trajectory createTrajectory(
    const tf2_ros::Buffer& tf_buffer, const geometry_msgs::PoseStamped& current_pose_global,
    const AstarWaypoints& astar_waypoints, const double& velocity) {
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = astar_waypoints.header.stamp;

  for (const auto& awp : astar_waypoints.waypoints) {
    autoware_planning_msgs::TrajectoryPoint point;

    point.pose = transformPose(awp.pose.pose, getTransform(tf_buffer, trajectory.header.frame_id,
                                                           awp.pose.header.frame_id));

    point.accel = {};
    point.twist = {};

    point.pose.position.z = current_pose_global.pose.position.z;  // height = const
    point.twist.linear.x = velocity / 3.6;                        // velocity = const

    // switch sign by forward/backward
    point.twist.linear.x = (awp.back ? -1 : 1) * point.twist.linear.x;

    trajectory.points.push_back(point);
  }

  return trajectory;
}

autoware_planning_msgs::Trajectory createStopTrajectory(
    const tf2_ros::Buffer& tf_buffer, const geometry_msgs::PoseStamped& current_pose_global) {
  AstarWaypoints waypoints;
  AstarWaypoint waypoint;

  waypoints.header.stamp = ros::Time::now();
  waypoints.header.frame_id = current_pose_global.header.frame_id;
  waypoint.pose.header = waypoints.header;
  waypoint.pose.pose = current_pose_global.pose;
  waypoint.back = false;
  waypoints.waypoints.push_back(waypoint);

  return createTrajectory(tf_buffer, current_pose_global, waypoints, 0.0);
}

}  // namespace

AstarNavi::AstarNavi() : nh_(), private_nh_("~"), tf_listener_(tf_buffer_) {
  private_nh_.param<double>("waypoints_velocity", waypoints_velocity_, 5.0);
  private_nh_.param<double>("update_rate", update_rate_, 1.0);
  private_nh_.param<double>("th_stopping_time_sec", th_stopping_time_sec_, 1.0);
  private_nh_.param<double>("th_stopping_distance_m", th_stopping_distance_m_, 1.0);
  private_nh_.param<double>("th_stopping_velocity_mps", th_stopping_velocity_mps_, 0.01);

  route_sub_ = private_nh_.subscribe("input/route", 1, &AstarNavi::onRoute, this);
  occupancy_grid_sub_ =
      private_nh_.subscribe("input/occupancy_grid", 1, &AstarNavi::onOccupancyGrid, this);
  scenario_sub_ = private_nh_.subscribe("input/scenario", 1, &AstarNavi::onScenario, this);
  twist_sub_ = private_nh_.subscribe("input/twist", 1, &AstarNavi::onTwist, this);

  trajectory_pub_ =
      private_nh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1, true);

  debug_pose_array_pub_ =
      private_nh_.advertise<geometry_msgs::PoseArray>("debug/pose_array", 1, true);

  debug_partial_pose_array_pub_ =
      private_nh_.advertise<geometry_msgs::PoseArray>("debug/partial_pose_array", 1, true);

  timer_ = private_nh_.createTimer(ros::Rate(update_rate_), &AstarNavi::onTimer, this);
}

void AstarNavi::onRoute(const autoware_planning_msgs::Route::ConstPtr& msg) {
  route_ = msg;

  goal_pose_global_.header = msg->header;
  goal_pose_global_.pose = msg->goal_pose;

  trajectory_ = {};
}

void AstarNavi::onOccupancyGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
  occupancy_grid_ = msg;
}

void AstarNavi::onScenario(const autoware_planning_msgs::Scenario::ConstPtr& msg) {
  scenario_ = msg;
}

void AstarNavi::onTwist(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  twist_ = msg;

  twist_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff = msg->header.stamp - twist_buffer_.front()->header.stamp;

    if (time_diff.toSec() < th_stopping_time_sec_) {
      break;
    }

    twist_buffer_.pop_front();
  }
}

bool AstarNavi::isPlanRequired() {
  if (trajectory_.points.empty()) {
    return true;
  }

  // TODO: obstacles

  // TODO: course out

  return false;
}

void AstarNavi::updateTarget() {
  const auto is_near_target =
      calculateDistance2d(trajectory_.points.at(target_index_).pose, current_pose_global_.pose) <
      th_stopping_distance_m_;

  const auto is_stopping = [&]() {
    for (const auto& twist : twist_buffer_) {
      if (std::abs(twist->twist.linear.x) > th_stopping_velocity_mps_) {
        return false;
      }
    }
    return true;
  }();

  if (is_near_target && is_stopping) {
    const auto new_target_index =
        getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);

    // Finished publishing all partial trajectories
    if (new_target_index == target_index_) {
      ROS_INFO("Astar completed");
      private_nh_.setParam("is_completed", true);
    } else {
      private_nh_.setParam("is_completed", false);

      // Switch to next partial trajectory
      prev_target_index_ = target_index_;
      target_index_ =
          getNextTargetIndex(trajectory_.points.size(), reversing_indices_, target_index_);
    }
  }
}

void AstarNavi::onTimer(const ros::TimerEvent& event) {
  // Check all inputs are ready
  if (!occupancy_grid_ || !route_ || !scenario_ || !twist_) {
    return;
  }

  if (!isActive(*scenario_)) {
    trajectory_ = {};
    return;
  }

  // Calculate global current pose
  current_pose_global_ = tf2pose(getTransform(tf_buffer_, "map", "base_link"));
  if (current_pose_global_.header.frame_id == "") {
    return;
  }

  if (isPlanRequired()) {
    planTrajectory();
  }

  updateTarget();

  const auto partial_trajectory =
      getPartialTrajectory(trajectory_, prev_target_index_, target_index_);

  trajectory_pub_.publish(partial_trajectory);

  // [debug] pose array
  {
    geometry_msgs::PoseArray pose_array;
    pose_array.header = trajectory_.header;

    for (const auto& point : trajectory_.points) {
      pose_array.poses.push_back(point.pose);
    }

    debug_pose_array_pub_.publish(pose_array);
  }

  // [debug] partial pose array
  {
    geometry_msgs::PoseArray partial_pose_array;
    partial_pose_array.header = partial_trajectory.header;

    for (const auto& point : partial_trajectory.points) {
      partial_pose_array.poses.push_back(point.pose);
    }

    debug_partial_pose_array_pub_.publish(partial_pose_array);
  }
}

void AstarNavi::planTrajectory() {
  // Calculate local goal pose
  goal_pose_local_.pose = transformPose(goal_pose_global_.pose,
                                        getTransform(tf_buffer_, occupancy_grid_->header.frame_id,
                                                     goal_pose_global_.header.frame_id));
  goal_pose_local_.header.frame_id = occupancy_grid_->header.frame_id;
  goal_pose_local_.header.stamp = goal_pose_global_.header.stamp;

  // Calculate local current pose
  current_pose_local_.pose = transformPose(
      current_pose_global_.pose, getTransform(tf_buffer_, occupancy_grid_->header.frame_id,
                                              current_pose_global_.header.frame_id));
  current_pose_local_.header.frame_id = occupancy_grid_->header.frame_id;
  current_pose_local_.header.stamp = current_pose_global_.header.stamp;

  // initialize vector for A* search, this runs only once
  AstarSearch astar;
  astar.initialize(*occupancy_grid_);

  // execute astar search
  const ros::WallTime start = ros::WallTime::now();
  const bool result = astar.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
  const ros::WallTime end = ros::WallTime::now();

  ROS_INFO("Astar planning: %f [s]", (end - start).toSec());

  if (result) {
    ROS_INFO("Found goal!");
    trajectory_ = createTrajectory(tf_buffer_, current_pose_global_, astar.getWaypoints(),
                                   waypoints_velocity_);
  } else {
    ROS_INFO("Can't find goal...");
    trajectory_ = createStopTrajectory(tf_buffer_, current_pose_global_);
  }

  reversing_indices_ = getReversingIndices(trajectory_);
  prev_target_index_ = 0;
  target_index_ =
      getNextTargetIndex(trajectory_.points.size(), reversing_indices_, prev_target_index_);
}

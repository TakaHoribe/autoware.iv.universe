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

AstarNavi::AstarNavi()
    : nh_(),
      private_nh_("~"),
      tf_listener_(tf_buffer_),
      occupancy_grid_initialized_(false),
      route_initialized_(false),
      current_pose_initialized_(false) {
  private_nh_.param<double>("waypoints_velocity", waypoints_velocity_, 5.0);
  private_nh_.param<double>("update_rate", update_rate_, 1.0);

  route_sub_ = private_nh_.subscribe("input/route", 1, &AstarNavi::onRoute, this);
  occupancy_grid_sub_ =
      private_nh_.subscribe("input/occupancy_grid", 1, &AstarNavi::onOccupancyGrid, this);

  trajectory_pub_ =
      private_nh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1, true);

  debug_pose_array_pub_ =
      private_nh_.advertise<geometry_msgs::PoseArray>("debug/pose_array", 1, true);

  timer_ = private_nh_.createTimer(ros::Rate(update_rate_), &AstarNavi::onTimer, this);
}

void AstarNavi::onOccupancyGrid(const nav_msgs::OccupancyGrid& msg) {
  occupancy_grid_ = msg;
  occupancy_grid_initialized_ = true;
}

void AstarNavi::onRoute(const autoware_planning_msgs::Route& msg) {
  if (!occupancy_grid_initialized_) {
    return;
  }

  goal_pose_global_.header = msg.header;
  goal_pose_global_.pose = msg.goal_pose;

  run();

  ros::Rate(update_rate_).sleep();
  route_initialized_ = true;
}

geometry_msgs::TransformStamped AstarNavi::getTransform(const std::string& from,
                                                        const std::string& to) {
  geometry_msgs::TransformStamped tf;
  try {
    tf = tf_buffer_.lookupTransform(from, to, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("[astar_navi] %s", ex.what());
  }
  return tf;
}

void AstarNavi::onTimer(const ros::TimerEvent& event) {
  if (!occupancy_grid_initialized_ || !route_initialized_) {
    return;
  }

  run();
}

void AstarNavi::run() {
  // Calculate local goal pose
  goal_pose_local_.pose = transformPose(
      goal_pose_global_.pose,
      getTransform(occupancy_grid_.header.frame_id, goal_pose_global_.header.frame_id));
  goal_pose_local_.header.frame_id = occupancy_grid_.header.frame_id;
  goal_pose_local_.header.stamp = goal_pose_global_.header.stamp;

  // Calculate global current pose
  geometry_msgs::TransformStamped tf_current_pose = getTransform("map", "base_link");

  current_pose_global_.header = tf_current_pose.header;
  current_pose_global_.pose.orientation = tf_current_pose.transform.rotation;
  current_pose_global_.pose.position.x = tf_current_pose.transform.translation.x;
  current_pose_global_.pose.position.y = tf_current_pose.transform.translation.y;
  current_pose_global_.pose.position.z = tf_current_pose.transform.translation.z;

  // Calculate local current pose
  current_pose_local_.pose = transformPose(
      current_pose_global_.pose,
      getTransform(occupancy_grid_.header.frame_id, current_pose_global_.header.frame_id));

  current_pose_local_.header.frame_id = occupancy_grid_.header.frame_id;
  current_pose_local_.header.stamp = current_pose_global_.header.stamp;

  current_pose_initialized_ = true;

  if (!occupancy_grid_initialized_ || !current_pose_initialized_ || !route_initialized_) {
    return;
  }

  // initialize vector for A* search, this runs only once
  astar_.initialize(occupancy_grid_);

  // execute astar search
  ros::WallTime start = ros::WallTime::now();
  bool result = astar_.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
  ros::WallTime end = ros::WallTime::now();

  ROS_INFO("Astar planning: %f [s]", (end - start).toSec());

  if (result) {
    ROS_INFO("Found GOAL!");
    publishTrajectory(astar_.getWaypoints(), waypoints_velocity_);
  } else {
    ROS_INFO("Can't find goal...");
    publishStopTrajectory();
  }

  astar_.reset();
}

void AstarNavi::publishTrajectory(const AstarWaypoints& astar_waypoints, const double& velocity) {
  autoware_planning_msgs::Trajectory trajectory;
  trajectory.header.frame_id = "map";
  trajectory.header.stamp = astar_waypoints.header.stamp;

  for (const auto& awp : astar_waypoints.waypoints) {
    autoware_planning_msgs::TrajectoryPoint point;

    point.pose = transformPose(awp.pose.pose,
                               getTransform(trajectory.header.frame_id, awp.pose.header.frame_id));

    point.accel = {};
    point.twist = {};

    point.pose.position.z = current_pose_global_.pose.position.z;  // height = const
    point.twist.linear.x = velocity / 3.6;                         // velocity = const

    // switch sign by forward/backward
    point.twist.linear.x = (awp.back ? -1 : 1) * point.twist.linear.x;

    trajectory.points.push_back(point);
  }

  trajectory_pub_.publish(trajectory);

  // debug
  geometry_msgs::PoseArray pose_array;
  pose_array.header = trajectory.header;

  for (const auto& point : trajectory.points) {
    pose_array.poses.push_back(point.pose);
  }

  debug_pose_array_pub_.publish(pose_array);
}

void AstarNavi::publishStopTrajectory() {
  AstarWaypoints waypoints;
  AstarWaypoint waypoint;

  waypoints.header.stamp = ros::Time::now();
  waypoints.header.frame_id = current_pose_global_.header.frame_id;
  waypoint.pose.header = waypoints.header;
  waypoint.pose.pose = current_pose_global_.pose;
  waypoint.back = false;
  waypoints.waypoints.push_back(waypoint);

  publishTrajectory(waypoints, 0.0);
}

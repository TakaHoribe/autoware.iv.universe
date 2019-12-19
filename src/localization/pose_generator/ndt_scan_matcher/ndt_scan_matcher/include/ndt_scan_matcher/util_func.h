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

#pragma once

#include <cmath>
#include <algorithm>

#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

//ref by http://takacity.blog.fc2.com/blog-entry-69.html
std_msgs::ColorRGBA ExchangeColorCrc(double x) {

  std_msgs::ColorRGBA color;

  x = std::max(x, 0.0);
  x = std::min(x, 0.9999);

  if (x <= 0.25) {
      color.b = 1.0;
      color.g = std::sin(x * 2.0 * M_PI);
      color.r = 0;
  }
  else if (x > 0.25 && x <= 0.5) {
      color.b = std::sin(x * 2 * M_PI);
      color.g = 1.0;
      color.r = 0;
  }
  else if (x > 0.5 && x <= 0.75) {
      color.b = 0;
      color.g = 1.0;
      color.r = -std::sin(x * 2.0 * M_PI);
  }
  else {
      color.b = 0;
      color.g = -std::sin(x * 2.0 * M_PI);
      color.r = 1.0;
  }
  color.a = 1.0;
  return color;
}

double calcDiffForRadian(const double lhs_rad, const double rhs_rad)
{
  double diff_rad = lhs_rad - rhs_rad;
  if (diff_rad > M_PI) {
    diff_rad = diff_rad - 2 * M_PI;
  } else if (diff_rad < -M_PI) {
    diff_rad = diff_rad + 2 * M_PI;
  }
  return diff_rad;
}

// x: roll, y: pitch, z: yaw
geometry_msgs::Vector3 getRPY(const geometry_msgs::PoseStamped &pose)
{
  geometry_msgs::Vector3 rpy;
  tf2::Quaternion q(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

geometry_msgs::Twist calcTwist(const geometry_msgs::PoseStamped &pose_a,
                               const geometry_msgs::PoseStamped &pose_b)
{
  const double dt = (pose_a.header.stamp - pose_b.header.stamp).toSec();

  if (dt == 0) {
    return geometry_msgs::Twist();
  }

  const auto pose_a_rpy = getRPY(pose_a);
  const auto pose_b_rpy = getRPY(pose_b);

  geometry_msgs::Vector3 diff_xyz;
  geometry_msgs::Vector3 diff_rpy;

  diff_xyz.x = pose_a.pose.position.x - pose_b.pose.position.x;
  diff_xyz.y = pose_a.pose.position.y - pose_b.pose.position.y;
  diff_xyz.z = pose_a.pose.position.z - pose_b.pose.position.z;
  diff_rpy.x = calcDiffForRadian(pose_a_rpy.x, pose_b_rpy.x);
  diff_rpy.y = calcDiffForRadian(pose_a_rpy.y, pose_b_rpy.y);
  diff_rpy.z = calcDiffForRadian(pose_a_rpy.z, pose_b_rpy.z);

  geometry_msgs::Twist twist;
  twist.linear.x = diff_xyz.x / dt;
  twist.linear.y = diff_xyz.y / dt;
  twist.linear.z = diff_xyz.z / dt;
  twist.angular.x = diff_rpy.x / dt;
  twist.angular.y = diff_rpy.y / dt;
  twist.angular.z = diff_rpy.z / dt;

  return twist;
}

geometry_msgs::PoseStamped interpolatePose(const geometry_msgs::PoseStamped &pose_a,
                            const geometry_msgs::PoseStamped &pose_b,
                            const ros::Time &time_stamp) {
  if (pose_a.header.stamp.toSec() == 0 || pose_b.header.stamp.toSec() == 0 || time_stamp.toSec() == 0) {
    return geometry_msgs::PoseStamped();
  }

  const auto twist = calcTwist(pose_a, pose_b);
  const double dt = (time_stamp - pose_a.header.stamp).toSec();

  const auto pose_a_rpy = getRPY(pose_a);

  geometry_msgs::Vector3 xyz;
  geometry_msgs::Vector3 rpy;
  xyz.x = pose_a.pose.position.x + twist.linear.x * dt;
  xyz.y = pose_a.pose.position.y + twist.linear.y * dt;
  xyz.z = pose_a.pose.position.z + twist.linear.z * dt;
  rpy.x = pose_a_rpy.x + twist.angular.x * dt;
  rpy.y = pose_a_rpy.y + twist.angular.y * dt;
  rpy.z = pose_a_rpy.z + twist.angular.z * dt;


  tf2::Quaternion tf_quaternion;
  tf_quaternion.setRPY(rpy.x, rpy.y, rpy.z);

  geometry_msgs::PoseStamped pose;
  pose.header = pose_a.header;
  pose.header.stamp = time_stamp;
  pose.pose.position.x = xyz.x;
  pose.pose.position.y = xyz.y;
  pose.pose.position.z = xyz.z;
  pose.pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}
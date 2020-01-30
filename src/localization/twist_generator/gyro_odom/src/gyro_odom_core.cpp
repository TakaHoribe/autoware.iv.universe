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

#include "gyro_odom/gyro_odom_core.h"

#include <cmath>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

GyroOdom::GyroOdom(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
{
  vehicle_twist_sub_ = nh_.subscribe("vehicle/twist", 100, &GyroOdom::callbackTwist, this);
  imu_sub_ = nh_.subscribe("imu", 100, &GyroOdom::callbackImu, this);

  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("twist", 10);
  // linear_x_pub_ = nh_.advertise<std_msgs::Float32>("linear_x", 10);
  // angular_z_pub_ = nh_.advertise<std_msgs::Float32>("angular_z", 10);
}

GyroOdom::~GyroOdom()
{
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
geometry_msgs::Vector3 getRPY(const geometry_msgs::Pose &pose)
{
  geometry_msgs::Vector3 rpy;
  tf2::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
  tf2::Matrix3x3(q).getRPY(rpy.x, rpy.y, rpy.z);
  return rpy;
}

geometry_msgs::Vector3 getRPY(const geometry_msgs::PoseStamped &pose)
{
  return getRPY(pose.pose);
}

void GyroOdom::callbackTwist(const geometry_msgs::TwistStamped::ConstPtr &twist_msg_ptr)
{
  twist_msg_ptr_ = twist_msg_ptr;
}

void GyroOdom::callbackImu(const sensor_msgs::Imu::ConstPtr &imu_msg_ptr)
{
  if(twist_msg_ptr_ == nullptr) {
    return;
  }

  geometry_msgs::TwistStamped twist;
  twist.header.stamp = imu_msg_ptr->header.stamp;
  twist.header.frame_id = "base_link";
  twist.twist.linear = twist_msg_ptr_->twist.linear;
  twist.twist.angular.z = -imu_msg_ptr->angular_velocity.z;  //TODO

  twist_pub_.publish(twist);
}

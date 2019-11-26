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

#include "convert_ros_msgs.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

geometry_msgs::PoseStamped convertToROSMsg(const std_msgs::Header &header,
                                           const Pose &pose) {
  tf2::Quaternion q;
  q.setRPY(pose.roll, pose.pitch, pose.yaw);

  geometry_msgs::PoseStamped msg;
  msg.header = header;
  msg.pose.position.x = pose.x;
  msg.pose.position.y = pose.y;
  msg.pose.position.z = pose.z;
  msg.pose.orientation.x = q.x();
  msg.pose.orientation.y = q.y();
  msg.pose.orientation.z = q.z();
  msg.pose.orientation.w = q.w();
  return msg;
}

geometry_msgs::PoseWithCovarianceStamped convertToROSMsg(const std_msgs::Header &header,
                                           const Pose &pose, const std::array<double, 36> cov_array) {
  tf2::Quaternion q;
  q.setRPY(pose.roll, pose.pitch, pose.yaw);

  geometry_msgs::PoseWithCovarianceStamped msg;
  msg.header = header;
  msg.pose.pose.position.x = pose.x;
  msg.pose.pose.position.y = pose.y;
  msg.pose.pose.position.z = pose.z;
  msg.pose.pose.orientation.x = q.x();
  msg.pose.pose.orientation.y = q.y();
  msg.pose.pose.orientation.z = q.z();
  msg.pose.pose.orientation.w = q.w();

  for(size_t i = 0; i < cov_array.size(); ++i) {
      msg.pose.covariance[i] = cov_array[i];
  }
  return msg;
}

geometry_msgs::TwistStamped convertToROSMsg(const std_msgs::Header &header,
                                            const Velocity &velocity) {
  geometry_msgs::TwistStamped msg;
  msg.header = header;
  msg.twist.linear.x = velocity.linear.x;
  msg.twist.linear.y = velocity.linear.y;
  msg.twist.linear.z = velocity.linear.z;
  msg.twist.angular.x = velocity.angular.x;
  msg.twist.angular.y = velocity.angular.y;
  msg.twist.angular.z = velocity.angular.z;
  return msg;
}

Pose convertFromROSMsg(const geometry_msgs::Pose &msg) {
  double roll, pitch, yaw;
  tf2::Quaternion orientation(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w);
  tf2::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

  Pose pose;
  pose.x = msg.position.x;
  pose.y = msg.position.y;
  pose.z = msg.position.z;
  pose.roll = roll;
  pose.pitch = pitch;
  pose.yaw = yaw;

  return pose;
}

Pose convertFromROSMsg(const geometry_msgs::PoseStamped &msg) {
  return convertFromROSMsg(msg.pose);
}

Pose convertFromROSMsg(const geometry_msgs::PoseWithCovarianceStamped &msg) {
  return convertFromROSMsg(msg.pose.pose);
}

Velocity convertFromROSMsg(const geometry_msgs::Twist &msg) {
  Velocity velocity;
  velocity.linear.x = msg.linear.x;
  velocity.linear.y = msg.linear.y;
  velocity.linear.z = msg.linear.z;
  velocity.angular.x = msg.angular.x;
  velocity.angular.y = msg.angular.y;
  velocity.angular.z = msg.angular.z;

  return velocity;
}

Velocity convertFromROSMsg(const geometry_msgs::TwistStamped &msg) {
  return convertFromROSMsg(msg.twist);
}

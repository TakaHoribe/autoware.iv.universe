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

#include <string>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <dynamic_reconfigure/server.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include "ndt_scan_matcher/NDTAlign.h"

class PoseInitializer {

public:
  PoseInitializer(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~PoseInitializer();

private:

  void callbackMapPoints(const sensor_msgs::PointCloud2::ConstPtr &pointcloud2_msg_ptr);
  bool serviceGNSSInitial(ndt_scan_matcher::NDTAlign::Request &req, ndt_scan_matcher::NDTAlign::Response &res);
  void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_conv_msg_ptr);

  geometry_msgs::PoseWithCovarianceStamped getHeight(const geometry_msgs::PoseWithCovarianceStamped &initial_pose_msg_ptr);
  geometry_msgs::PoseWithCovarianceStamped callAlignService(const geometry_msgs::PoseWithCovarianceStamped &msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber initial_pose_sub_;
  ros::Subscriber map_points_sub_;

  ros::Publisher initial_pose_pub_;

  ros::ServiceClient ndt_client_;

  ros::ServiceServer gnss_service_;

  // dynamic_reconfigure::Server<ndt_slam::PoseInitializerConfig> server_;
  // dynamic_reconfigure::Server<ndt_slam::PoseInitializerConfig>::CallbackType f_;


  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_ptr_;
  std::string map_frame_;

};

/*
 * Copyright 2020 TierIV. All rights reserved.
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
#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl_ros/transforms.h>
#include <memory>

#include "lidar_apollo_instance_segmentation/debugger.h"

class LidarInstanceSegmentationInterface
{
public:
  LidarInstanceSegmentationInterface() {}
  virtual ~LidarInstanceSegmentationInterface() {}
  virtual bool detectDynamicObjects(
    const sensor_msgs::PointCloud2 & input,
    autoware_perception_msgs::DynamicObjectWithFeatureArray & output) = 0;
};

class LidarInstanceSegmentationNode
{
private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher dynamic_objects_pub_;
  std::shared_ptr<LidarInstanceSegmentationInterface> detector_ptr_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string target_frame_;
  float z_offset_;
  Debugger debugger_;
  void pointCloudCallback(const sensor_msgs::PointCloud2 & msg);

public:
  LidarInstanceSegmentationNode();
  ~LidarInstanceSegmentationNode() {}
};

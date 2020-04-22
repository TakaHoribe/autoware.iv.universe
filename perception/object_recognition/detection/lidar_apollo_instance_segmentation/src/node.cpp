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

#include "lidar_apollo_instance_segmentation/node.h"
#include "lidar_apollo_instance_segmentation/detector.h"

LidarInstanceSegmentationNode::LidarInstanceSegmentationNode() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  detector_ptr_ = std::make_shared<LidarApolloInstanceSegmentation>();
  pointcloud_sub_ =
    pnh_.subscribe("input/pointcloud", 1, &LidarInstanceSegmentationNode::pointCloudCallback, this);
  dynamic_objects_pub_ = pnh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>(
    "output/labeled_clusters", 1);
  pnh_.param<std::string>("target_frame", target_frame_, "base_link");
  pnh_.param<float>("z_offset", z_offset_, 2);
}

void LidarInstanceSegmentationNode::pointCloudCallback(const sensor_msgs::PointCloud2 & msg)
{
  autoware_perception_msgs::DynamicObjectWithFeatureArray output_msg;

  // transform pointcloud to tagret_frame
  sensor_msgs::PointCloud2 transformed_pointcloud;
  if (target_frame_ != msg.header.frame_id) {
    try {
      geometry_msgs::TransformStamped transform_stamped;
      transform_stamped = tf_buffer_.lookupTransform(target_frame_, msg.header.frame_id,
                                                     msg.header.stamp, ros::Duration(0.5));
      Eigen::Matrix4f affine_matrix =
        tf2::transformToEigen(transform_stamped.transform).matrix().cast<float>();
      pcl_ros::transformPointCloud(affine_matrix, msg, transformed_pointcloud);
      transformed_pointcloud.header.frame_id = target_frame_;
    } catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
    }
  } else {
    transformed_pointcloud = msg;
  }

  // move up pointcloud +z_offset in z axis
  sensor_msgs::PointCloud2 pointcloud_with_z_offset;
  Eigen::Affine3f z_up_translation(Eigen::Translation3f(0, 0, z_offset_));
  Eigen::Matrix4f z_up_transform = z_up_translation.matrix();
  pcl_ros::transformPointCloud(z_up_transform, transformed_pointcloud, pointcloud_with_z_offset);

  detector_ptr_->detectDynamicObjects(pointcloud_with_z_offset, output_msg);

  // move down pointcloud z_offset in z axis
  Eigen::Affine3f z_down_translation(Eigen::Translation3f(0, 0, -z_offset_));
  Eigen::Matrix4f z_down_transform = z_down_translation.matrix();
  for (int i=0; i<output_msg.feature_objects.size(); i++) {
    sensor_msgs::PointCloud2 transformed_pointcloud;
    pcl_ros::transformPointCloud(z_down_transform, output_msg.feature_objects.at(i).feature.cluster, transformed_pointcloud);
    output_msg.feature_objects.at(i).feature.cluster = transformed_pointcloud;
  }

  dynamic_objects_pub_.publish(output_msg);
  debugger_.publishColoredPointCloud(output_msg);
}

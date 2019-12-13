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

#include "pose_initializer/pose_initializer_core.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

double getGroundHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr pcdmap, const tf2::Vector3& point)
{
    constexpr double radius = 1.0 * 1.0;
    const double x = point.getX();
    const double y = point.getY();

    double height = INFINITY;
    for(const auto& p : pcdmap->points)
    {
        const double dx = x - p.x;
        const double dy = y - p.y;
        const double sd = (dx * dx) + (dy * dy);
        if(sd < radius)
        {
            height = std::min(height, static_cast<double>(p.z));
        }
    }
    return std::isfinite(height) ? height : point.getZ();
}

PoseInitializer::PoseInitializer(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , tf2_listener_(tf2_buffer_)
{
  initial_pose_sub_ = nh_.subscribe("initialpose", 10, &PoseInitializer::callbackInitialPose, this);
  initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose3d", 10);

  ndt_client_ = nh_.serviceClient<ndt_scan_matcher::NDTAlign>("ndt_align_srv");
}

PoseInitializer::~PoseInitializer()
{
}
// void PoseInitializer::configCallback(const ndt_slam::PoseInitializerConfig &config, uint32_t level)
// {
// }

void PoseInitializer::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr)
{

  std::string fixed_frame = initial_pose_msg_ptr->header.frame_id;

  tf2::Vector3 point(initial_pose_msg_ptr->pose.pose.position.x, initial_pose_msg_ptr->pose.pose.position.y, initial_pose_msg_ptr->pose.pose.position.z);

  const auto pcdmsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("pointcloud_map", ros::Duration(3.0));

  pcl::PointCloud<pcl::PointXYZ>::Ptr pcdmap_(new pcl::PointCloud<pcl::PointXYZ>);
  if(pcdmsg)
  {
      std::string pcdmap_frame_ = pcdmsg->header.frame_id;
      pcl::fromROSMsg(*pcdmsg, *pcdmap_);

      tf2_ros::Buffer tf_buffer;
      tf2_ros::TransformListener tf_listener(tf_buffer);
      tf2::Transform transform;
      try
      {
          const auto stamped = tf_buffer.lookupTransform(pcdmap_frame_, fixed_frame, ros::Time(0), ros::Duration(1.0));
          tf2::fromMsg(stamped.transform, transform);
      }
      catch (tf2::TransformException& exception)
      {
          ROS_WARN_STREAM("failed to lookup transform: " << exception.what());
      }

      point = transform * point;
      point.setZ(getGroundHeight(pcdmap_, point));
      point = transform.inverse() * point;
  }

  geometry_msgs::PoseWithCovarianceStamped msg;
  msg = *initial_pose_msg_ptr;
  msg.pose.pose.position.x = point.getX();
  msg.pose.pose.position.y = point.getY();
  msg.pose.pose.position.z = point.getZ();

  ndt_scan_matcher::NDTAlign srv;
  srv.request.initial_pose_with_cov = msg;

  ros::Duration(1.0).sleep();  // NOTE: need rosbag replay
  ndt_client_.waitForExistence(ros::Duration(1.0));
  if(ndt_client_.call(srv))
  {
    std::cout << srv.response.result_pose_with_cov.pose.pose.position.x << std::endl;
    initial_pose_pub_.publish(srv.response.result_pose_with_cov);
  }
  else
  {
    ROS_ERROR("F*************************K");
  }
}

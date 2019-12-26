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
  , map_frame_("map")
{
  initial_pose_sub_ = nh_.subscribe("initialpose", 10, &PoseInitializer::callbackInitialPose, this);
  map_points_sub_ = nh_.subscribe("pointcloud_map", 1, &PoseInitializer::callbackMapPoints, this);

  initial_pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose3d", 10);

  ndt_client_ = nh_.serviceClient<autoware_localization_srvs::PoseWithCovarianceStamped>("ndt_align_srv");
  ndt_client_.waitForExistence(ros::Duration(1.0)); //TODO

  gnss_service_ = nh.advertiseService("pose_initializer_srv", &PoseInitializer::serviceInitial, this);

}

PoseInitializer::~PoseInitializer()
{
}

void PoseInitializer::callbackMapPoints(const sensor_msgs::PointCloud2::ConstPtr &map_points_msg_ptr)
{
  std::string map_frame_ = map_points_msg_ptr->header.frame_id;
  map_ptr_ = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_ptr_);
}

bool PoseInitializer::serviceInitial(autoware_localization_srvs::PoseWithCovarianceStamped::Request &req, autoware_localization_srvs::PoseWithCovarianceStamped::Response &res)
{
  const auto a = getHeight(req.pose_with_cov);
  const auto b = callAlignService(a);


  initial_pose_pub_.publish(b);

  return true;
}

void PoseInitializer::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr)
{
  const auto a = getHeight(*initial_pose_msg_ptr);
  auto b = callAlignService(a);
  // NOTE temporary cov
  b.pose.covariance[0] = 1.0;
  b.pose.covariance[1*6+1] = 1.0;
  b.pose.covariance[2*6+2] = 0.01;
  b.pose.covariance[3*6+3] = 0.01;
  b.pose.covariance[4*6+4] = 0.01;
  b.pose.covariance[5*6+5] = 1.5;

  initial_pose_pub_.publish(b);
}

geometry_msgs::PoseWithCovarianceStamped PoseInitializer::getHeight(const geometry_msgs::PoseWithCovarianceStamped &initial_pose_msg_ptr)
{
  std::string fixed_frame = initial_pose_msg_ptr.header.frame_id;
  tf2::Vector3 point(initial_pose_msg_ptr.pose.pose.position.x, initial_pose_msg_ptr.pose.pose.position.y, initial_pose_msg_ptr.pose.pose.position.z);

  if(map_ptr_)
  {
      tf2::Transform transform;
      try
      {
          const auto stamped = tf2_buffer_.lookupTransform(map_frame_, fixed_frame, ros::Time(0), ros::Duration(1.0));
          tf2::fromMsg(stamped.transform, transform);
      }
      catch (tf2::TransformException& exception)
      {
          ROS_WARN_STREAM("failed to lookup transform: " << exception.what());
      }

      point = transform * point;
      point.setZ(getGroundHeight(map_ptr_, point));
      point = transform.inverse() * point;
  }

  geometry_msgs::PoseWithCovarianceStamped msg;
  msg = initial_pose_msg_ptr;
  msg.pose.pose.position.x = point.getX();
  msg.pose.pose.position.y = point.getY();
  msg.pose.pose.position.z = point.getZ();
  return msg;
}

geometry_msgs::PoseWithCovarianceStamped PoseInitializer::callAlignService(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  autoware_localization_srvs::PoseWithCovarianceStamped srv;
  srv.request.pose_with_cov = msg;

  ROS_INFO("[pose_initializer] call NDT Align Server");
  if(ndt_client_.call(srv))
  {
    ROS_INFO("[pose_initializer] called NDT Align Server");
    // NOTE temporary cov
    srv.response.pose_with_cov.pose.covariance[0] = 1.0;
    srv.response.pose_with_cov.pose.covariance[1*6+1] = 1.0;
    srv.response.pose_with_cov.pose.covariance[2*6+2] = 0.01;
    srv.response.pose_with_cov.pose.covariance[3*6+3] = 0.01;
    srv.response.pose_with_cov.pose.covariance[4*6+4] = 0.01;
    srv.response.pose_with_cov.pose.covariance[5*6+5] = 0.2;
  }
  else
  {
    ROS_ERROR("[pose_initializer] could not call NDT Align Server");
  }

  return srv.response.pose_with_cov;
}





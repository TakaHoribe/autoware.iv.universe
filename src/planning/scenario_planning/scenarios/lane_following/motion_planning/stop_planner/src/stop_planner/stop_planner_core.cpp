
/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include "stop_planner/stop_planner_core.hpp"

StopPlanner::StopPlanner()
    : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  trajectory_pub_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1, true);
  pcd_extructed_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("extructed/pcd", 1, true);
  marker_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/markers", 1);

  trajectory_sub_ = pnh_.subscribe("input/trajectory", 1, &StopPlanner::callbackTrajectory, this);
  obstacle_pcd_sub_ = pnh_.subscribe("input/point_cloud", 1, &StopPlanner::callbackPointCloud, this);

  double wheel_base, tread, stop_margin_dist, detection_area_width;
  int object_pcd_threshold_num;
  pnh_.param("/vehicle_info/wheel_base", wheel_base, double(2.79));
  pnh_.param("/vehicle_info/wheel_tread", tread, double(2.0));
  pnh_.param("stop_margin_dist", stop_margin_dist, double(7.0));
  pnh_.param("object_pcd_threshold_num", object_pcd_threshold_num, int(1));
  pnh_.param("detection_area_width", detection_area_width, double(2.5));
  obstacle_pcd_velocity_planner_.setVehicleWidth(tread);
  obstacle_pcd_velocity_planner_.setStopDistance(stop_margin_dist);
  obstacle_pcd_velocity_planner_.setPointsThreshold(object_pcd_threshold_num);
  obstacle_pcd_velocity_planner_.setDetectionAreaWidth(detection_area_width);



  /* wait until base_link is received */
  while (ros::ok())
  {
    if (updateCurrentPose(10.0))
    {
      break;
    }
    else
    {
      ROS_INFO("[StopPlanner] waiting base_link at initialization");
    }
  }
}

StopPlanner::~StopPlanner() {}

void StopPlanner::callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
  in_obstacle_pcd_ptr_ = std::make_shared<sensor_msgs::PointCloud2>(*msg);
}

void StopPlanner::callbackTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr &msg)
{
  in_trajectory_ptr_ = std::make_shared<autoware_planning_msgs::Trajectory>(*msg);

  if (!updateCurrentPose(0.0))
  {
    ROS_INFO_DELAYED_THROTTLE(2.0, "[StopPlanner] fail to get base_link.");
    return;
  }

  /* nullptr guard */
  if (in_trajectory_ptr_ == nullptr || current_pose_ptr_ == nullptr)
  {
    ROS_INFO_DELAYED_THROTTLE(2.0, "[StopPlanner] waiting topics. trajectory  %d, current_pose = %d",
             in_trajectory_ptr_ != nullptr, current_pose_ptr_ != nullptr);
    return;
  }

  /* if obstacle pcd is not received */
  if (in_obstacle_pcd_ptr_ == nullptr)
  {
    ROS_DEBUG_DELAYED_THROTTLE(2.0, "[StopPlanner] pcd is not received.");
    trajectory_pub_.publish(*in_trajectory_ptr_);
    return;
  }

  /* convert pcd */
  geometry_msgs::TransformStamped transform_stamped;
  try
  {
    transform_stamped = tf_buffer_.lookupTransform(/*target*/ "map",/*src*/ in_obstacle_pcd_ptr_->header.frame_id, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("%s", ex.what());
    ROS_WARN("target = %s, in_obstacle_pcd_ptr_->header.frame_id = %s\n",
            "map", in_obstacle_pcd_ptr_->header.frame_id.c_str());
    return;
  }
  sensor_msgs::PointCloud2 transformed_pointcloud;
  tf2::doTransform(*in_obstacle_pcd_ptr_, transformed_pointcloud, transform_stamped);


  /* obstacle stop point calculation */
  auto ts = std::chrono::system_clock::now();

  obstacle_pcd_velocity_planner_.setCurrentPose(current_pose_ptr_->pose);
  obstacle_pcd_velocity_planner_.setCurrentTrajectory(*in_trajectory_ptr_);
  obstacle_pcd_velocity_planner_.setPointCloud(transformed_pointcloud);
  autoware_planning_msgs::Trajectory out_trajectory = obstacle_pcd_velocity_planner_.run();

  sensor_msgs::PointCloud2 pcd_extructed;
  obstacle_pcd_velocity_planner_.getExtructedPcd(pcd_extructed);
  pcd_extructed.header.stamp = ros::Time::now();
  pcd_extructed.header.frame_id = "map";

  auto tf = std::chrono::system_clock::now();
  double calctime_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(tf - ts).count() * 1.0e-6;
  // ROS_INFO("obstacle processing time : %lf [ms]", calctime_ms);

  trajectory_pub_.publish(out_trajectory);
  pcd_extructed_pub_.publish(pcd_extructed);
  marker_viz_pub_.publish(obstacle_pcd_velocity_planner_.visualize());


  return;
}

bool StopPlanner::updateCurrentPose(const double timeout)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform("map", "base_link",ros::Time(0), ros::Duration(timeout));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_INFO_DELAYED_THROTTLE(3.0, "[StopPlanner] cannot get map to base_link transform. %s", ex.what());
    return false;
  }

  geometry_msgs::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(ps);
  return true;
};

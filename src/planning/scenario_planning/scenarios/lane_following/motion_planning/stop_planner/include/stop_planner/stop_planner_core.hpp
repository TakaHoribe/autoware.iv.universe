
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

#ifndef MOTION_PLANNING_STOP_PLANNER_H
#define MOTION_PLANNING_STOP_PLANNER_H

#include <ros/ros.h>

#include <autoware_planning_msgs/Trajectory.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include "stop_planner/obstacle_pcd_velocity_planner.h"

class StopPlanner
{
public:
  StopPlanner();
  ~StopPlanner();

private:
  void trajecotyCallback();

  // ros
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher trajectory_pub_;
  ros::Publisher marker_viz_pub_;
  ros::Publisher pcd_extructed_pub_;
  ros::Subscriber trajectory_sub_;
  ros::Subscriber obstacle_pcd_sub_;

  tf2_ros::Buffer tf_buffer_;              //!< @brief tf buffer for current_pose
  tf2_ros::TransformListener tf_listener_; //!< @brief tf listener for current_pose

  std::shared_ptr<sensor_msgs::PointCloud2> in_obstacle_pcd_ptr_;
  std::shared_ptr<autoware_planning_msgs::Trajectory> in_trajectory_ptr_;
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;

  ObstaclePcdVelocityPlanner obstacle_pcd_velocity_planner_;

  void callbackTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr &msg);
  void callbackPointCloud(const sensor_msgs::PointCloud2::ConstPtr &msg);
  bool updateCurrentPose(const double timeout);
};

#endif
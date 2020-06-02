/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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

#include <vector>

#include <geometry_msgs/TwistStamped.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <tf2/utils.h>

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/Trajectory.h>

namespace motion_planning
{
class AdaptiveCruiseController
{
public:
  AdaptiveCruiseController(
    const double vehicle_width, const double vehicle_length, const double wheel_base,
    const double front_overhang);

  void insertAdaptiveCruiseVelocity(
    const autoware_planning_msgs::Trajectory & trajectory, const int nearest_collision_point_idx,
    const geometry_msgs::Pose self_pose, const pcl::PointXYZ & nearest_collision_point,
    const ros::Time nearest_collision_point_time,
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr object_ptr,
    const geometry_msgs::TwistStamped::ConstPtr current_velocity_ptr, bool * need_to_stop,
    autoware_planning_msgs::Trajectory * output_trajectory);

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  /*
   * Parameter
   */
  double vehicle_width_;
  double vehicle_length_;
  double wheel_base_;
  double front_overhang_;

  ros::Time prev_collsion_point_time_;
  pcl::PointXYZ prev_collsion_point_;
  double prev_target_vehicle_time_ = 0.0;
  double prev_target_vehicle_dist_ = 0.0;
  bool prev_collsion_point_valid_ = false;
  std::vector<geometry_msgs::TwistStamped> est_vel_que_;

  struct Param
  {
    double stop_margin;
    double min_behavior_stop_margin;

    //!< @brief use tracking objects for estimating object velocity or not
    bool use_object_to_estimate_vel;

    //!< @brief use pcl for estimating object velocity or not
    bool use_pcl_to_estimate_vel;

    //!< @brief consider forward vehicle velocity to self upper velocity or not
    bool consider_obj_velocity;

    static constexpr double object_length_margin = 2.0;
    static constexpr double object_width_margin = 0.5;
    static constexpr double valid_est_vel_dif_time = 1.0;
    static constexpr double valid_est_vel_max = 20;
    static constexpr double valid_est_vel_min = -20;
    static constexpr double valid_vel_que_time = 0.5;
    static constexpr double thresh_vel_to_stop = 0.5;

    /* parameter for acc */
    //!< @brief threshold of forward obstacle velocity to insert stop line (to stop acc)
    static constexpr double obstacle_stop_velocity_thresh = 1.0;

    //!< @brief supposed minimum acceleration in emergency stop
    static constexpr double emergency_stop_acceleration = -5.0;

    //!< @brief supposed idling time to start emergency stop
    static constexpr double emergency_stop_idling_time = 0.5;

    //!< @brief minimum distance of emergency stop
    static constexpr double min_dist_stop = 4.0;

    //!< @brief supposed maximum acceleration in active cruise control
    static constexpr double max_standard_acceleration = 0.5;

    //!< @brief supposed minimum acceleration(deceleration) in active cruise control
    static constexpr double min_standard_acceleration = -1.0;

    //!< @brief supposed idling time to react object in active cruise control
    static constexpr double standard_idling_time = 0.5;

    //!< @brief supposed idling time to stop active cruise control
    static constexpr double min_dist_standard = 4.0;

    //!< @brief supposed minimum acceleration of forward obstacle
    static constexpr double obstacle_min_standard_acceleration = -1.5;

    //!< @brief margin to insert upper velocity
    static constexpr double margin_rate_to_change_vel = 0.3;

    /* parameter for pid used in acc */
    //!< @brief coefficient P in PID control (used when target dist -current_dist >=0)
    double p_coeff_pos;

    //!< @brief coefficient P in PID control (used when target dist -current_dist <0)
    double p_coeff_neg;

    //!< @brief coefficient P in PID control (used when delta_dist >=0)
    double d_coeff_pos;

    //!< @brief coefficient P in PID control (used when delta_dist <0)
    double d_coeff_neg;

    static constexpr double d_coeff_valid_time = 1.0;
    static constexpr double d_coeff_valid_diff_vel = 20.0;
    static constexpr double d_max_vel_norm = 3.0;
  };
  Param param_;

  double getMedianVel(const std::vector<geometry_msgs::TwistStamped> vel_que);
  void calcDistanceToNearestPointOnPath(
    const autoware_planning_msgs::Trajectory & trajectory, const int nearest_point_idx,
    const geometry_msgs::Pose & self_pose, const pcl::PointXYZ & nearest_collision_point,
    double * distance);
  double calcTrajYaw(
    const autoware_planning_msgs::Trajectory & trajectory, const int collsion_point_idx);
  bool estimatePointVelocityFromObject(
    const autoware_perception_msgs::DynamicObjectArray::ConstPtr object_ptr, const double traj_yaw,
    const pcl::PointXYZ & nearest_collision_point, double * velocity);
  bool estimatePointVelocityFromPcl(
    const double traj_yaw, const pcl::PointXYZ & nearest_collision_point,
    const ros::Time & nearest_collision_point_time, double * velocity);
  double calcUpperVelocity(const double dist_to_col, const double obj_vel, const double self_vel);
  double calcThreshDistToForwardObstacle(const double current_vel, const double obj_vel);
  double calcBaseDistToForwardObstacle(const double current_vel, const double obj_vel);
  double calcTargetVelocity_P(const double target_dist, const double current_dist);
  double calcTargetVelocity_I(const double target_dist, const double current_dist);
  double calcTargetVelocity_D(const double target_dist, const double current_dist);
  double calcTargetVelocityByPID(
    const double current_vel, const double current_dist, const double obj_vel);

  void insertMaxVelocityToPath(
    const double current_vel, const double target_vel, const double dist_to_collsion_point,
    autoware_planning_msgs::Trajectory * output_trajectory);
  void registerQueToVelocity(const double vel, const ros::Time & vel_time);
};

}  // namespace motion_planning

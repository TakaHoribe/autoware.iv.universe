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

#include <deque>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

#include "libplanning_utils/planning_utils.h"
#include "libplanning_utils/pure_pursuit.h"

enum class Mode : int32_t {
  waypoint,
  dialog,
};

template <class T>
typename std::underlying_type<T>::type enumToInteger(T t) {
  return static_cast<typename std::underlying_type<T>::type>(t);
}

struct PurePursuitDynamicConfig {
  int32_t param_flag_;
  double const_lookahead_distance_;
  double const_velocity_;
  double lookahead_distance_ratio_;
  double minimum_lookahead_distance_;
  double reverse_minld_;  // minimum lookahead distance at reverse gear

  PurePursuitDynamicConfig() {
    param_flag_ = enumToInteger(Mode::dialog);
    const_lookahead_distance_ = 4.0;
    const_velocity_ = 5.0;
    lookahead_distance_ratio_ = 1.5;
    minimum_lookahead_distance_ = 4.0;
    reverse_minld_ = 6.0;
  }
};

class PurePursuitNode {
 public:
  PurePursuitNode();

 private:
  // Node Handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // Subscriber
  ros::Subscriber sub_trajectory_;
  ros::Subscriber sub_current_velocity_;

  // Callback Function
  void onTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr& msg);
  void onCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg);

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  void updateCurrentPose();

  // Publisher
  ros::Publisher pub_twist_cmd_;
  ros::Publisher pub_ctrl_cmd_;

  // Debug Publisher
  ros::Publisher pub_ld_;
  ros::Publisher pub_le_;
  ros::Publisher pub_viz_;
  ros::Publisher pub_cv_;
  ros::Publisher pub_lec_;
  ros::Publisher pub_sdc_;

  // Debug Publish Function
  void publishCommand(const double kappa, const double cmd_vel, const double cmd_acc);
  void publishZeroCommand();
  void publishTwistStamped(const double kappa, const double cmd_vel);
  void publishControlCommandStamped(const double kappa, const double cmd_vel, const double cmd_acc);
  void publishLateralError(const double lat_error);
  void publishLateralErrorCompensation(const double lat_error_comp_ratio);

  void publishCurrentCommandVelocity(const double cmd_vel);

  void publishVisualizer(const geometry_msgs::Point& next_wp_pos, const geometry_msgs::Point& next_tgt_pos,
                         const geometry_msgs::Pose& curr_pose, const double ld);
  void publishLookaheadDistance(const double ld);

  // Timer
  ros::Timer timer_;
  void onTimer(const ros::TimerEvent& event);

  std::unique_ptr<planning_utils::PurePursuit> pp_ptr_;
  std::unique_ptr<PurePursuitDynamicConfig> ppdconf_ptr_;

  // Parameter
  bool use_lerp_;
  bool publish_twist_cmd_;
  bool publish_ctrl_cmd_;
  bool use_lat_error_compensation_;
  bool use_steering_delay_compensation_;
  double wheel_base_;
  double ctrl_period_;
  double velocity_delay_compensation_time_;
  double lec_ratio_, lec_max_;
  double steering_delay_compensation_time_;

  // Variable
  std::deque<double> angular_z_buffer_;  // [rad/s]
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;
  autoware_planning_msgs::Trajectory::ConstPtr current_trajectory_ptr_;
  geometry_msgs::TwistStamped::ConstPtr current_velocity_ptr_;
};

// functions
double computeLookaheadDistance(const PurePursuitDynamicConfig& ppdconf, double curr_linear_vel, double cmd_vel);
double computeCommandVelocity(const PurePursuitDynamicConfig& ppdconf,
                              const std::vector<autoware_planning_msgs::TrajectoryPoint>& points, int32_t clst_wp_idx);
double computeCommandAcceleration(const PurePursuitDynamicConfig& ppdconf,
                                  const std::vector<autoware_planning_msgs::TrajectoryPoint>& points,
                                  int32_t clst_wp_idx);
autoware_planning_msgs::TrajectoryPoint computeTargetPointsWithDelayCompensation(
    const PurePursuitDynamicConfig& ppdconf, const std::vector<autoware_planning_msgs::TrajectoryPoint>& points,
    const int32_t clst_wp_idx, const double& delay, const double& curr_vel);
double computeCommandVelocityWithDelayCompensation(const PurePursuitDynamicConfig& ppdconf,
                                                   const std::vector<autoware_planning_msgs::TrajectoryPoint>& points,
                                                   const int32_t clst_wp_idx, const double& delay,
                                                   const double& curr_vel);
double computeCommandAccelerationWithDelayCompensation(
    const PurePursuitDynamicConfig& ppdconf, const std::vector<autoware_planning_msgs::TrajectoryPoint>& points,
    const int32_t clst_wp_idx, const double& delay, const double& curr_vel);
double computeLateralError(const geometry_msgs::Point& point,
                           const std::vector<autoware_planning_msgs::TrajectoryPoint>& points, int32_t clst_wp_idx);

geometry_msgs::Pose computePoseWithSteeringDelayCompensation(const geometry_msgs::Pose& curr_pose, double curr_linear_x,
                                                             const std::deque<double>& curr_angular_z_buf,
                                                             double time_delta);

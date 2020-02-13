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

#ifndef PURE_PURSUIT_CORE_H
#define PURE_PURSUIT_CORE_H

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

// User defined includes
#include <autoware_health_checker/health_checker/health_checker.h>
#include "autoware_config_msgs/ConfigWaypointFollower.h"
#include "autoware_msgs/ControlCommandStamped.h"
#include "autoware_planner_msgs/Trajectory.h"
#include "libplanning_utils/planning_utils.h"
#include "libplanning_utils/pure_pursuit.h"

// C++ includes
#include <chrono>
#include <memory>

namespace waypoint_follower {

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
  ~PurePursuitNode() = default;

 private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // publisher
  ros::Publisher twist_pub_, ctrl_pub_, ld_pub_, le_pub_, viz_pub_, cv_pub_, lec_pub_, sdc_pub_;

  // subscriber
  ros::Subscriber pose_sub_, traj_sub_, vel_sub_, config_sub_;

  // processing Timer
  ros::Timer proc_timer_;

  std::unique_ptr<planning_utils::PurePursuit> pp_ptr_;
  std::unique_ptr<PurePursuitDynamicConfig> ppdconf_ptr_;

  // variables
  bool pub_twist_cmd_, pub_ctrl_cmd_, use_lat_error_compensation_, use_steering_delay_compensation_;
  double wheel_base_;
  double ctrl_period_;
  double velocity_delay_compensation_time_;
  double lec_ratio_, lec_max_;
  double steering_delay_compensation_time_;
  std::deque<double> angular_z_buffer_;  //[rad/s]

  geometry_msgs::TwistStampedConstPtr curr_vel_ptr_;
  autoware_planner_msgs::TrajectoryConstPtr curr_traj_ptr_;
  geometry_msgs::PoseStampedConstPtr curr_pose_ptr_;

  // initializer
  void initForROS();

  // callbacks
  void timerCallback(const ros::TimerEvent& event);
  void configCallback(const autoware_config_msgs::ConfigWaypointFollowerConstPtr& config);
  void cpCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void cvCallback(const geometry_msgs::TwistStampedConstPtr& msg);
  void trajCallback(const autoware_planner_msgs::TrajectoryConstPtr& msg);

  // publsihers
  void publishCommand(double kappa, double cmd_vel, double cmd_acc);
  void publishZeroCommand();
  void publishTwistStamped(double kappa, double cmd_vel);
  void publishControlCommandStamped(double kappa, double cmd_vel, double cmd_acc);
  void publishLateralError(double lat_error);
  void publishLateralErrorCompensation(double lat_error_comp_ratio);

  void publishCurrentCommandVelocity(double cmd_vel);

  void publishVisualizer(const geometry_msgs::Point& next_wp_pos, const geometry_msgs::Point& next_tgt_pos,
                         const geometry_msgs::Pose& curr_pose, double ld);
  void publishLookaheadDistance(double ld);
};

// functions
double computeLookaheadDistance(const PurePursuitDynamicConfig& ppdconf, double curr_linear_vel, double cmd_vel);
double computeCommandVelocity(const PurePursuitDynamicConfig& ppdconf,
                              const std::vector<autoware_planner_msgs::MotionStamped>& curr_mos, int32_t clst_wp_idx);
double computeCommandAcceleration(const PurePursuitDynamicConfig& ppdconf,
                                  const std::vector<autoware_planner_msgs::MotionStamped>& curr_mos,
                                  int32_t clst_wp_idx);
double computeCommandAccelerationWithDelayCompensation(
    const PurePursuitDynamicConfig& ppdconf, const std::vector<autoware_planner_msgs::MotionStamped>& curr_mos,
    const int32_t clst_wp_idx, const double& delay, const double& curr_vel);
double computeLateralError(const geometry_msgs::Point& point,
                           const std::vector<autoware_planner_msgs::MotionStamped>& curr_mos, int32_t clst_wp_idx);

geometry_msgs::Pose computePoseWithSteeringDelayCompensation(const geometry_msgs::Pose& curr_pose, double curr_linear_x,
                                                             const std::deque<double>& curr_angular_z_buf,
                                                             double time_delta);

}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_CORE_H

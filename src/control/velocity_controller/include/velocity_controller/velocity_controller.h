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

#ifndef VELOCITY_CONTROLLER
#define VELOCITY_CONTROLLER

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_planning_msgs/Trajectory.h>

#include "delay_compensation.h"
#include "pid.h"
#include "lowpass_filter.h"
#include "velocity_controller_mathutils.h"

class VelocityController
{
public:
  VelocityController();
  ~VelocityController() = default;

private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber sub_current_velocity_, sub_trajectory_, sub_is_sudden_stop_;
  ros::Publisher pub_control_cmd_;
  ros::Publisher pub_debug_;
  ros::Timer timer_control_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_; //!< @brief tf listener

  // parameters
  // enabled flags
  bool use_sudden_stop_, use_smooth_stop_, use_delay_compensation_, use_velocity_feedback_, use_acceleration_limit_,
      use_jerk_limit_, use_slope_compensation_;
  bool show_debug_info_; //!< @brief flag to show debug info
  bool use_pub_debug_;

  // timer callback
  double control_rate_;

  // dt
  double max_dt_; //!< @brief max value of dt
  double min_dt_; //!< @brief min value of dt

  // closest waypoint
  double distance_threshold_closest_, angle_threshold_closest_;

  // stop state
  double stop_state_velocity_, stop_state_acceleration_;
  double current_velocity_threshold_stop_state_;
  double target_velocity_threshold_stop_state_;

  // sudden stop
  double sudden_stop_velocity_, sudden_stop_acceleration_;
  double max_jerk_sudden_stop_, min_jerk_sudden_stop_;

  // delay compensation
  double delay_compensation_time_;

  // emergency stop
  double emergency_stop_velocity_;
  double emergency_stop_acceleration_;
  double max_jerk_emergency_stop_;
  double min_jerk_emergency_stop_;

  // smooth stop
  double velocity_threshold_drive_;
  double current_velocity_threshold_high_smooth_stop_, current_velocity_threshold_low_smooth_stop_,
      target_velocity_threshold_high_smooth_stop_, target_velocity_threshold_low_smooth_stop_;
  double weak_brake_time_, increasing_brake_time_, stop_brake_time_;
  double weak_brake_acceleration_, increasing_brake_gradient_, stop_brake_acceleration_;

  // acceleration limit
  double max_acceleration_, min_acceleration_;

  // jerk limit
  double max_jerk_, min_jerk_;

  // slope compensation
  double max_pitch_rad_, min_pitch_rad_;
  double lpf_pitch_gain_;

  // velocity feedback
  double kp_pid_velocity_;
  double ki_pid_velocity_;
  double kd_pid_velocity_;
  double max_ret_pid_velocity_;
  double min_ret_pid_velocity_;
  double max_p_pid_velocity_;
  double min_p_pid_velocity_;
  double max_d_pid_velocity_;
  double min_d_pid_velocity_;
  double max_i_pid_velocity_;
  double min_i_pid_velocity_;
  double current_velocity_threshold_pid_integrate_;
  double lpf_velocity_error_gain_;

  // variables
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> current_velocity_ptr_;
  std::shared_ptr<autoware_planning_msgs::Trajectory> trajectory_ptr_;

  // calculate dt
  bool first_time_control_;
  ros::Time prev_control_time_;

  // shift mode
  enum Shift
  {
    Forward = 0,
    Reverse,
  } prev_shift_;

  // sudden stop
  bool is_sudden_stop_;

  // smooth stop
  bool is_smooth_stop_, drive_after_smooth_stop_;
  bool is_emergency_stop_smooth_stop_;
  ros::Time start_time_smooth_stop_;

  // jerk limit
  double prev_acceleration_;

  // slope compensation
  Lpf1d lpf_pitch;

  // velocity feedback
  PIDController pid_velocity_;
  Lpf1d lpf_velocity_error;

  // debug topic
  std_msgs::Float32MultiArray debug_values_;
  unsigned int num_debug_values_ = 18;

  /**
   * [0]: dt
   * [1]: current velocity
   * [2]: target velocity
   * [3]: target acceleration after delay compensation
   * [4]: shift
   * [5]: pitch after LPF [rad]
   * [6]: error velocity after LPF
   * [7]: mode (0: init check, 1: PID, 2: Stop, 3: Sudden stop, 4: Smooth stop, 5: Closest waypoint error, 6: Emergency stop) 
   * [8]: acceleration after PID
   * [9]: acceleration after acceleration limit
   * [10]: acceleration after jerk limit
   * [11]: acceleration after slope compensation
   * [12]: published acceleration
   * [13]: raw pitch [rad]
   * [14]: raw error velocity
   * [15]: pitch after LPF [deg]
   * [16]: raw pitch [deg]
   * [17]: closest waypoint target acceleration
   **/

  // methods
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr &msg);
  void callbackIsSuddenStop(const std_msgs::Bool msg);
  void callbackTimerControl(const ros::TimerEvent &event);

  void publishControlCommandStamped(const double velocity, const double acceleration);
  void publishDebugValues();

  double getDt();
  bool updateCurrentPose(const double timeout_sec);
  bool getCurretPoseFromTF(const double timeout_sec, geometry_msgs::PoseStamped &ps);

  enum Shift getCurrentShiftMode(const double target_velocity, const double target_acceleration);
  double getPitch(const geometry_msgs::Quaternion &quaternion) const;
  double calculateAccelerationSmoothStop();
  double applyAccelerationLimitFilter(const double acceleration, const double max_acceleration,
                                      const double min_acceleration);
  double applyJerkLimitFilter(const double acceleration, const double dt, const double max_jerk, const double min_jerk);
  double applySlopeCompensation(const double acceleration, const double pitch, const Shift shift);
  double applyVelocityFeedback(const double target_acceleration, const double error_velocity, const double dt,
                               const bool is_integrated);
  void resetSmoothStop();
  void resetEmergencyStop();

  void writeDebugValuesParameters(const double dt, const double current_velocity, const double target_velocity,
                                  const double target_acceleration, const Shift shift, const double pitch,
                                  const double error_velocity, const int32_t closest_waypoint_index);

  void writeDebugValuesCmdAcceleration(const double cmd_acceleration, const double mode);
  void resetDebugValues();
};

#endif

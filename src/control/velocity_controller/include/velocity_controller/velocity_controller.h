/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not enable this file except in compliance with the License.
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

#include <memory>

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
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_current_velocity_;
  ros::Subscriber sub_trajectory_;
  ros::Subscriber sub_is_sudden_stop_;
  ros::Publisher pub_control_cmd_;
  ros::Publisher pub_debug_;
  ros::Timer timer_control_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_; //!< @brief tf listener

  // parameters
  // enabled flags
  bool show_debug_info_; //!< @brief flag to show debug info
  bool enable_sudden_stop_;
  bool enable_smooth_stop_;
  bool enable_delay_compensation_;
  bool enable_slope_compensation_;

  // timer callback
  double control_rate_;

  // closest waypoint
  double closest_dist_thr_;
  double closest_angle_thr_;

  // stop state
  double stop_state_velocity_;
  double stop_state_acceleration_;
  double stop_state_entry_ego_speed_;
  double stop_state_entry_target_speed_;

  // sudden stop
  double sudden_stop_velocity_;
  double sudden_stop_acceleration_;
  double sudden_stop_max_jerk_;
  double sudden_stop_min_jerk_;

  // delay compensation
  double delay_compensation_time_;

  // emergency stop
  double emergency_stop_acc_lim_;
  double emergency_stop_jerk_lim_;

  // smooth stop
  double velocity_threshold_drive_;
  struct SmoothStopParam
  {
    double exit_ego_speed;
    double entry_ego_speed;
    double exit_target_speed;
    double entry_target_speed;
    double weak_brake_time;
    double weak_brake_acc;
    double increasing_brake_gradient;
    double increasing_brake_time;
    double stop_brake_time;
    double stop_brake_acc;
  } smooth_stop_param_;


  // acceleration limit
  double max_acceleration_;
  double min_acceleration_;

  // jerk limit
  double max_jerk_;
  double min_jerk_;

  // slope compensation
  double max_pitch_rad_;
  double min_pitch_rad_;

  // velocity feedback
  double current_velocity_threshold_pid_integrate_;

  // variables
  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> current_velocity_ptr_;
  std::shared_ptr<autoware_planning_msgs::Trajectory> trajectory_ptr_;

  // calculate dt
  std::shared_ptr<ros::Time> prev_control_time_;

  // shift mode
  enum Shift
  {
    Forward = 0,
    Reverse,
  } prev_shift_;

  // sudden stop
  bool is_sudden_stop_;

  // smooth stop
  bool is_smooth_stop_;
  bool drive_after_smooth_stop_;
  bool is_emergency_stop_smooth_stop_;
  ros::Time start_time_smooth_stop_;

  // jerk limit
  double prev_acceleration_;

  // slope compensation
  Lpf1d lpf_pitch;

  // velocity feedback
  PIDController pid_velocity_;
  Lpf1d lpf_velocity_error;

  // methods
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg);
  void callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr &msg);
  void callbackIsSuddenStop(const std_msgs::Bool msg);
  void callbackTimerControl(const ros::TimerEvent &event);

  void publishCtrlCmd(const double velocity, const double acceleration, const int controller_mode);

  double getDt();
  bool updateCurrentPose(const double timeout_sec);
  bool getCurretPoseFromTF(const double timeout_sec, geometry_msgs::PoseStamped &ps);
  double calcInterpolatedTargetVelocity(const autoware_planning_msgs::Trajectory &traj, const geometry_msgs::PoseStamped &curr_pose,
                                        const double curr_vel, const int closest);

  enum Shift getCurrentShiftMode(const double target_velocity, const double target_acceleration);
  double getPitch(const geometry_msgs::Quaternion &quaternion) const;
  double calcSmoothStopAcc();
  double applyAccFilter(const double acceleration, const double max_acceleration,
                                      const double min_acceleration);
  double applyJerkFilter(const double acceleration, const double dt, const double max_jerk, const double min_jerk);
  double applySlopeCompensation(const double acceleration, const double pitch, const Shift shift);
  double applyVelocityFeedback(const double target_acceleration, const double error_velocity, const double dt,
                               const bool is_integrated);
  void resetSmoothStop();
  void resetEmergencyStop();

  /* Debug */
  std_msgs::Float32MultiArray debug_values_;
  unsigned int num_debug_values_ = 21;

  /**
   * [0]: dt
   * [1]: current velocity
   * [2]: target velocity
   * [3]: target acceleration after delay compensation
   * [4]: shift
   * [5]: pitch after LPF [rad]
   * [6]: error velocity after LPF
   * [7]: controller_mode (0: init check, 1: PID, 2: Stop, 3: Sudden stop, 4: Smooth stop, 5: Closest waypoint error, 6: Emergency stop) 
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
  void writeDebugValues(const double dt, const double current_velocity, const double target_vel, const double target_acc,
                        const Shift shift, const double pitch, const double error_velocity, const int32_t closest_waypoint_index);
};

#endif

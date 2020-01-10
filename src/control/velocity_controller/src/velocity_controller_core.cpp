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

#include "velocity_controller.h"

// clang-format off
#define DEBUG_INFO(...) { if(show_debug_info_) { ROS_INFO(__VA_ARGS__); } }

// clang-format on

VelocityController::VelocityController() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_), first_time_control_(true),
                                           is_sudden_stop_(false), is_smooth_stop_(false), drive_after_smooth_stop_(false),
                                           is_emergency_stop_smooth_stop_(false), prev_acceleration_(0.0)
{
  // parameters
  // parameters for subscriber, publisher and timer
  pnh_.param("control_rate", control_rate_, double(30.0));

  // parameters to enable functions
  pnh_.param("use_sudden_stop", use_sudden_stop_, bool(true));
  pnh_.param("use_smooth_stop", use_smooth_stop_, bool(true));
  pnh_.param("use_delay_compensation", use_delay_compensation_, bool(true));
  pnh_.param("use_velocity_feedback", use_velocity_feedback_, bool(true));
  pnh_.param("use_acceleration_limit", use_acceleration_limit_, bool(true));
  pnh_.param("use_jerk_limit", use_jerk_limit_, bool(true));
  pnh_.param("use_slope_compensation", use_slope_compensation_, bool(true));
  pnh_.param("show_debug_info", show_debug_info_, bool(false));

  // parameters for calculating dt
  pnh_.param("max_dt", max_dt_, double((1.0 / control_rate_) * 2.0)); // [sec]
  pnh_.param("min_dt", min_dt_, double((1.0 / control_rate_) / 2.0)); // [sec]

  // parameters to find a closest waypoint
  pnh_.param("distance_threshold_closest_waypoint", distance_threshold_closest_, double(3.0));
  pnh_.param("angle_threshold_closest_waypoint", angle_threshold_closest_, double(M_PI_4));

  // parameters for stop state
  pnh_.param("stop_state_velocity", stop_state_velocity_, double(0.0));                                     // [m/s]
  pnh_.param("stop_state_acceleration", stop_state_acceleration_, double(-2.0));                            // [m/s^2]
  pnh_.param("current_velocity_threshold_stop_state", current_velocity_threshold_stop_state_, double(0.2)); // [m/s]
  pnh_.param("target_velocity_threshold_stop_state", target_velocity_threshold_stop_state_, double(0.1));   // [m/s]

  // parameters for sudden stop
  pnh_.param("sudden_stop_velocity", sudden_stop_velocity_, double(0.0));          // [m/s]
  pnh_.param("sudden_stop_acceleration", sudden_stop_acceleration_, double(-2.0)); // [m/s^2]
  pnh_.param("max_jerk_sudden_stop", max_jerk_sudden_stop_, double(0.0));          // [m/s^3]
  pnh_.param("min_jerk_sudden_stop", min_jerk_sudden_stop_, double(-1.5));         // [m/s^3]

  // parameters for delay compensation
  pnh_.param("delay_compensation_time", delay_compensation_time_, double(0.17)); // [sec]

  // parameters for emergency stop by this controller
  pnh_.param("emergency_stop_velocity", emergency_stop_velocity_, double(0.0));          // [m/s]
  pnh_.param("emergency_stop_acceleration", emergency_stop_acceleration_, double(-2.0)); // [m/s^2]
  pnh_.param("max_jerk_emergency_stop", max_jerk_emergency_stop_, double(0.0));          // [m/s^3]
  pnh_.param("min_jerk_emergency_stop", min_jerk_emergency_stop_, double(-1.5));         // [m/s^3]

  // parameters for smooth stop
  pnh_.param("velocity_threshold_drive", velocity_threshold_drive_, double(2.2)); // [m/s]
  pnh_.param("current_velocity_threshold_high_smooth_stop", current_velocity_threshold_high_smooth_stop_,
             double(2.0)); // [m/s]
  pnh_.param("current_velocity_threshold_low_smooth_stop", current_velocity_threshold_low_smooth_stop_,
             double(1.0)); // [m/s]
  pnh_.param("target_velocity_threshold_high_smooth_stop", target_velocity_threshold_high_smooth_stop_,
             double(2.0)); // [m/s]
  pnh_.param("target_velocity_threshold_low_smooth_stop", target_velocity_threshold_low_smooth_stop_,
             double(1.0));                                                            // [m/s]
  pnh_.param("weak_brake_time", weak_brake_time_, double(3.0));                       // [sec]
  pnh_.param("increasing_brake_time", increasing_brake_time_, double(3.0));           // [sec]
  pnh_.param("stop_brake_time", stop_brake_time_, double(2.0));                       // [sec]
  pnh_.param("weak_brake_acceleration", weak_brake_acceleration_, double(-0.4));      // [m/s^2]
  pnh_.param("increasing_brake_gradient", increasing_brake_gradient_, double(-0.05)); // [m/s^3]
  pnh_.param("stop_brake_acceleration", stop_state_acceleration_, double(-1.7));      // [m/s^2]

  // parameters for acceleration limit
  pnh_.param("max_acceleration", max_acceleration_, double(2.0));  // [m/s^2]
  pnh_.param("min_acceleration", min_acceleration_, double(-5.0)); // [m/s^2]

  // parameters for jerk limit
  pnh_.param("max_jerk", max_jerk_, double(2.0));  // [m/s^3]
  pnh_.param("min_jerk", min_jerk_, double(-5.0)); // [m/s^3]

  // parameters for slope compensation
  pnh_.param("max_pitch_rad", max_pitch_rad_, double(0.1));  // [rad]
  pnh_.param("min_pitch_rad", min_pitch_rad_, double(-0.1)); // [rad]

  pnh_.param("current_velocity_threshold_pid_integrate", current_velocity_threshold_pid_integrate_, double(0.5)); // [m/s]

  // subscriber, publisher and timer
  sub_current_velocity_ = pnh_.subscribe("current_velocity", 1, &VelocityController::callbackCurrentVelocity, this);
  sub_trajectory_ = pnh_.subscribe("current_trajectory", 1, &VelocityController::callbackTrajectory, this);
  sub_is_sudden_stop_ = pnh_.subscribe("is_sudden_stop", 1, &VelocityController::callbackIsSuddenStop, this);
  pub_control_cmd_ = pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("control_cmd", 1);
  pub_debug_ = pnh_.advertise<std_msgs::Float32MultiArray>("debug_values", 1);
  timer_control_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &VelocityController::callbackTimerControl, this);

  // initialize PID gain
  double kp, ki, kd;
  pnh_.param("kp_pid_velocity", kp, double(0.0));
  pnh_.param("ki_pid_velocity", ki, double(0.0));
  pnh_.param("kd_pid_velocity", kd, double(0.0));
  pid_velocity_.setGains(kp, ki, kd);

  // initialize PID limits
  double max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d;
  pnh_.param("max_ret_pid_velocity", max_pid, double(0.0)); // [m/s^2]
  pnh_.param("min_ret_pid_velocity", min_pid, double(0.0)); // [m/s^2]
  pnh_.param("max_p_pid_velocity", max_p, double(0.0));     // [m/s^2]
  pnh_.param("min_p_pid_velocity", min_p, double(0.0));     // [m/s^2]
  pnh_.param("max_i_pid_velocity", max_i, double(0.0));     // [m/s^2]
  pnh_.param("min_i_pid_velocity", min_i, double(0.0));     // [m/s^2]
  pnh_.param("max_d_pid_velocity", max_d, double(0.0));     // [m/s^2]
  pnh_.param("min_d_pid_velocity", min_d, double(0.0));     // [m/s^2]
  pid_velocity_.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);

  // set lowpass filter
  double lpf_pitch_gain;
  pnh_.param("lpf_pitch_gain", lpf_pitch_gain, double(0.95));
  lpf_pitch.init(lpf_pitch_gain);

  double lpf_velocity_error_gain;
  pnh_.param("lpf_velocity_error_gain", lpf_velocity_error_gain, double(0.9));
  lpf_velocity_error.init(lpf_velocity_error_gain);

  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);

  /* wait to get vehicle position */
  while (ros::ok())
  {
    if (!updateCurrentPose(5.0))
    {
      ROS_INFO("[velocity_controller] waiting map to base_link at initialize.");
    }
    else
    {
      break;
    }
  }
}

void VelocityController::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  current_velocity_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
}

void VelocityController::callbackTrajectory(const autoware_planning_msgs::TrajectoryConstPtr &msg)
{
  trajectory_ptr_ = std::make_shared<autoware_planning_msgs::Trajectory>(*msg);
}

void VelocityController::callbackIsSuddenStop(const std_msgs::Bool msg)
{
  is_sudden_stop_ = msg.data;
}

bool VelocityController::getCurretPoseFromTF(const double timeout_sec, geometry_msgs::PoseStamped &ps)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(
        "map",       /* targert */
        "base_link", /* src */
        ros::Time(0), ros::Duration(timeout_sec));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_DELAYED_THROTTLE(3.0, "[velocity_controller] cannot get map to base_link transform. %s", ex.what());
    return false;
  }
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  return true;
}

bool VelocityController::updateCurrentPose(const double timeout_sec)
{
  geometry_msgs::PoseStamped ps;
  if (!getCurretPoseFromTF(timeout_sec, ps))
  {
    return false;
  }
  current_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(ps);
  return true;
}

void VelocityController::callbackTimerControl(const ros::TimerEvent &event)
{
  resetDebugValues();

  if (!updateCurrentPose(0.0))
  {
    return;
  }

  // init check
  if (current_pose_ptr_ == nullptr || current_velocity_ptr_ == nullptr || trajectory_ptr_ == nullptr)
  {
    DEBUG_INFO("[VC Init Check] Waiting topics, Publish stop command. pose: %d, velocity: %d, trajectory: %d",
               (int)(current_pose_ptr_ != nullptr), (int)(current_velocity_ptr_ != nullptr), (int)(trajectory_ptr_ != nullptr));
    publishControlCommandStamped(stop_state_velocity_, stop_state_acceleration_);
    writeDebugValuesCmdAcceleration(stop_state_acceleration_, 0.0);
    publishDebugValues();
    return;
  }

  // initialize parameters
  const double dt = getDt();

  // find a closest waypoint index
  int closest_idx;
  if (!velocity_controller_mathutils::calcClosestWithThr(*trajectory_ptr_, current_pose_ptr_->pose,
                                                         angle_threshold_closest_, distance_threshold_closest_, closest_idx))
  {
    ROS_ERROR_DELAYED_THROTTLE(5.0, "[VC Find Closest Waypoint] cannot find closest! Emergency Stop! (dist_thr = %3.3f [m], angle_thr = %3.3f [rad])",
                               distance_threshold_closest_, angle_threshold_closest_);
    double cmd_acceleration =
        applyJerkLimitFilter(sudden_stop_acceleration_, dt, max_jerk_sudden_stop_, min_jerk_sudden_stop_);
    publishControlCommandStamped(sudden_stop_velocity_, cmd_acceleration);
    writeDebugValuesCmdAcceleration(cmd_acceleration, 5.0);
    publishDebugValues();
    return;
  }

  double current_velocity = current_velocity_ptr_->twist.linear.x;
  double target_velocity = trajectory_ptr_->points.at(closest_idx).twist.linear.x;
  double target_acceleration;
  // delay compensation
  if (use_delay_compensation_)
  {
    target_acceleration = DelayCompensator::computeCommandAccelerationAfterDelayTime(
        *trajectory_ptr_, closest_idx, delay_compensation_time_,
        current_velocity_ptr_->twist.linear.x);
  }
  else
  {
    target_acceleration = trajectory_ptr_->points.at(closest_idx).accel.linear.x;
  }

  const Shift shift = getCurrentShiftMode(target_velocity, target_acceleration);
  if (shift != prev_shift_)
  {
    pid_velocity_.reset();
  }
  prev_shift_ = shift;
  const double pitch = lpf_pitch.filter(getPitch(current_pose_ptr_->pose.orientation));
  const double error_velocity = lpf_velocity_error.filter(target_velocity - current_velocity);
  DEBUG_INFO("[VC Init params] current_vel = %f, target_vel = %f, target_acc = %f, pitch = %f, error_velocity = %f",
             current_velocity, target_velocity, target_acceleration, pitch, error_velocity);
  writeDebugValuesParameters(dt, current_velocity, target_velocity, target_acceleration, shift, pitch, error_velocity,
                             closest_idx);

  // sudden stop
  if (use_sudden_stop_ && is_sudden_stop_)
  {
    double cmd_acceleration =
        applyJerkLimitFilter(sudden_stop_acceleration_, dt, max_jerk_sudden_stop_, min_jerk_sudden_stop_);
    publishControlCommandStamped(sudden_stop_velocity_, cmd_acceleration);
    pid_velocity_.reset();
    resetSmoothStop();

    DEBUG_INFO("[VC Sudden Stop] STOP!");
    writeDebugValuesCmdAcceleration(cmd_acceleration, 3.0);
    publishDebugValues();
    return;
  }

  // stop state
  if (std::fabs(current_velocity) < current_velocity_threshold_stop_state_ &&
      std::fabs(target_velocity) < target_velocity_threshold_stop_state_ && !is_smooth_stop_)
  {
    double cmd_acceleration = applyAccelerationLimitFilter(stop_state_acceleration_, max_acceleration_, min_acceleration_);
    cmd_acceleration = applyJerkLimitFilter(cmd_acceleration, dt, max_jerk_, min_jerk_);
    cmd_acceleration = applySlopeCompensation(cmd_acceleration, pitch, shift);
    publishControlCommandStamped(stop_state_velocity_, cmd_acceleration);
    pid_velocity_.reset();
    resetSmoothStop();
    resetEmergencyStop();

    DEBUG_INFO("[VC Stop State] Stopping");
    writeDebugValuesCmdAcceleration(cmd_acceleration, 2.0);
    publishDebugValues();
    return;
  }

  // emergency stop by this controller
  if (is_emergency_stop_smooth_stop_)
  {
    double cmd_acceleration =
        applyJerkLimitFilter(emergency_stop_acceleration_, dt, max_jerk_emergency_stop_, min_jerk_emergency_stop_);
    publishControlCommandStamped(emergency_stop_velocity_, cmd_acceleration);
    pid_velocity_.reset();
    resetSmoothStop();

    ROS_WARN_DELAYED_THROTTLE(3.0, "[VC Emergency Stop] STOP!");
    writeDebugValuesCmdAcceleration(cmd_acceleration, 6.0);
    publishDebugValues();
    return;
  }

  // smooth stop
  if (use_smooth_stop_)
  {
    DEBUG_INFO("[VC Smooth stop] is_smooth_stop = %d, drive_after_stop = %d", is_smooth_stop_, drive_after_smooth_stop_);
    // If the current or target velocity increases during smooth stopping, exit smooth stopping
    if (is_smooth_stop_ && (std::fabs(current_velocity) > current_velocity_threshold_high_smooth_stop_))
    {
      is_smooth_stop_ = false;
      is_emergency_stop_smooth_stop_ = true;
    }

    if (is_smooth_stop_ && (std::fabs(target_velocity) > target_velocity_threshold_high_smooth_stop_))
    {
      is_smooth_stop_ = false;
    }

    if (std::fabs(current_velocity) < current_velocity_threshold_low_smooth_stop_ &&
        std::fabs(target_velocity) < target_velocity_threshold_low_smooth_stop_ && drive_after_smooth_stop_)
    {
      is_smooth_stop_ = true;
      drive_after_smooth_stop_ = false;
      start_time_smooth_stop_ = ros::Time::now();
    }

    if (is_smooth_stop_)
    {
      double cmd_acceleration = calculateAccelerationSmoothStop();
      cmd_acceleration = applySlopeCompensation(cmd_acceleration, pitch, shift);
      cmd_acceleration = applyAccelerationLimitFilter(cmd_acceleration, max_acceleration_, min_acceleration_);
      cmd_acceleration = applyJerkLimitFilter(cmd_acceleration, dt, max_jerk_, min_jerk_);
      publishControlCommandStamped(target_velocity, cmd_acceleration);
      pid_velocity_.reset();
      DEBUG_INFO("[VC Smooth stop] Smooth stopping. cmd_acc = %f", cmd_acceleration);
      writeDebugValuesCmdAcceleration(cmd_acceleration, 4.0);
      publishDebugValues();
      return;
    }

    if (std::fabs(current_velocity) > velocity_threshold_drive_)
    {
      drive_after_smooth_stop_ = true;
    }
  }

  // velocity feedback
  if (use_velocity_feedback_)
  {
    DEBUG_INFO("[VC Velocity Feedback] current_vel = %f, target_vel = %f, target_acc = %f", current_velocity,
               target_velocity, target_acceleration);
    // if (shift == Shift::Reverse)
    // {
    //   current_velocity = -current_velocity;
    //   target_velocity = -target_velocity;
    //   target_acceleration = -target_acceleration;
    // }
    const bool enable_integration = std::fabs(current_velocity) < current_velocity_threshold_pid_integrate_ ? false : true;

    double cmd_acceleration = applyVelocityFeedback(target_acceleration, error_velocity, dt, enable_integration);
    cmd_acceleration = applySlopeCompensation(cmd_acceleration, pitch, shift);
    cmd_acceleration = applyAccelerationLimitFilter(cmd_acceleration, max_acceleration_, min_acceleration_);
    cmd_acceleration = applyJerkLimitFilter(cmd_acceleration, dt, max_jerk_, min_jerk_);

    DEBUG_INFO("[VC Velocity Feedback] after PID and filters cmd_acc = %f", cmd_acceleration);
    publishControlCommandStamped(target_velocity, cmd_acceleration);
    writeDebugValuesCmdAcceleration(cmd_acceleration, 1.0);
    publishDebugValues();
    return;
  }
}

void VelocityController::publishControlCommandStamped(const double velocity, const double acceleration)
{
  prev_acceleration_ = acceleration;

  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "base_link";
  cmd.control.velocity = velocity;
  cmd.control.acceleration = acceleration;
  pub_control_cmd_.publish(cmd);
}

void VelocityController::publishDebugValues()
{
  pub_debug_.publish(debug_values_);
}

double VelocityController::getDt()
{
  double dt;
  if (first_time_control_)
  {
    dt = 1.0 / control_rate_;
    prev_control_time_ = ros::Time::now();
    first_time_control_ = false;
  }
  else
  {
    dt = (ros::Time::now() - prev_control_time_).toSec();
    prev_control_time_ = ros::Time::now();
  }
  dt = std::max(std::min(dt, max_dt_), min_dt_);
  return dt;
}

enum VelocityController::Shift VelocityController::getCurrentShiftMode(const double target_velocity,
                                                                       const double target_acceleration)
{
  if (target_velocity > 0)
  {
    return Shift::Forward;
  }
  else if (target_velocity < 0)
  {
    return Shift::Reverse;
  }
  else
  {
    return prev_shift_;
  }
}

double VelocityController::getPitch(const geometry_msgs::Quaternion &quaternion) const
{
  Eigen::Quaterniond q(quaternion.w, quaternion.x, quaternion.y, quaternion.z);
  Eigen::Vector3d v = q.toRotationMatrix() * Eigen::Vector3d::UnitX();
  double den = std::max(std::sqrt(v.x() * v.x() + v.y() * v.y()), 1.0E-8 /* avoid 0 divide */);
  double pitch = (-1.0) * std::atan2(v.z(), den);
  return pitch;
}

double VelocityController::calculateAccelerationSmoothStop()
{
  const double elapsed_time = (ros::Time::now() - start_time_smooth_stop_).toSec();

  double cmd_acceleration;
  if (elapsed_time < weak_brake_time_)
  {
    cmd_acceleration = weak_brake_acceleration_;
    DEBUG_INFO("[VC Smooth Stop] weak breaking! acc = %f", cmd_acceleration);
  }
  else if (elapsed_time < (weak_brake_time_ + increasing_brake_time_))
  {
    const double dt = elapsed_time - weak_brake_time_;
    cmd_acceleration = weak_brake_acceleration_ + increasing_brake_gradient_ * dt;
    DEBUG_INFO("[VC Smooth Stop] break increasing! acc = %f", cmd_acceleration);
  }
  else if (elapsed_time < (weak_brake_time_ + increasing_brake_time_ + stop_brake_time_))
  {
    cmd_acceleration = stop_brake_acceleration_;
    DEBUG_INFO("[VC Smooth Stop] stop breaking! acc = %f", cmd_acceleration);
  }
  else
  {
    is_smooth_stop_ = false;
    cmd_acceleration = stop_brake_acceleration_;
    DEBUG_INFO("[VC Smooth Stop] finish smooth stopping! acc = %f", cmd_acceleration);
  }

  return cmd_acceleration;
}

double VelocityController::applyAccelerationLimitFilter(const double acceleration, const double max_acceleration,
                                                        const double min_acceleration)
{
  if (use_acceleration_limit_)
  {
    const double filtered_acceleration = std::min(std::max(acceleration, min_acceleration), max_acceleration);

    debug_values_.data.at(9) = filtered_acceleration;

    return filtered_acceleration;
  }
  else
  {
    return acceleration;
  }
}

double VelocityController::applyJerkLimitFilter(const double acceleration, const double dt, const double max_jerk,
                                                const double min_jerk)
{
  if (use_jerk_limit_)
  {
    const double jerk = (acceleration - prev_acceleration_) / dt;
    const double filtered_jerk = std::min(std::max(jerk, min_jerk), max_jerk);
    const double filtered_acceleration = prev_acceleration_ + (filtered_jerk * dt);

    debug_values_.data.at(10) = filtered_acceleration;

    return filtered_acceleration;
  }
  else
  {
    return acceleration;
  }
}

double VelocityController::applySlopeCompensation(const double acceleration, const double pitch,
                                                  const Shift shift)
{
  if (use_slope_compensation_)
  {
    const double gravity = 9.80665;
    const double pitch_limited = std::min(std::max(pitch, min_pitch_rad_), max_pitch_rad_);
    double compensated_acceleration;
    if (shift == Shift::Forward)
    {
      compensated_acceleration = acceleration - gravity * std::sin(pitch_limited);
    }
    else if (shift == Shift::Reverse)
    {
      compensated_acceleration = acceleration + gravity * std::sin(pitch_limited);
    }
    else
    {
      compensated_acceleration = acceleration;
    }

    debug_values_.data.at(11) = compensated_acceleration;

    return compensated_acceleration;
  }
  else
  {
    return acceleration;
  }
}

double VelocityController::applyVelocityFeedback(const double target_acceleration, const double error_velocity,
                                                 const double dt, const bool enable_integration)
{
  if (use_velocity_feedback_)
  {
    std::vector<double> pid_contributions(3);
    double feedbacked_acceleration =
        target_acceleration + pid_velocity_.calculate(error_velocity, dt, enable_integration, pid_contributions);

    debug_values_.data.at(8) = feedbacked_acceleration;
    debug_values_.data.at(18) = pid_contributions.at(0);
    debug_values_.data.at(19) = pid_contributions.at(1);
    debug_values_.data.at(20) = pid_contributions.at(2);

    return feedbacked_acceleration;
  }
  else
  {
    return target_acceleration;
  }
}

void VelocityController::resetSmoothStop()
{
  is_smooth_stop_ = false;
  drive_after_smooth_stop_ = false;
}

void VelocityController::resetEmergencyStop()
{
  is_emergency_stop_smooth_stop_ = false;
}

void VelocityController::writeDebugValuesParameters(const double dt, const double current_velocity,
                                                    const double target_velocity, const double target_acceleration,
                                                    const Shift shift, const double pitch, const double error_velocity,
                                                    const int32_t closest_waypoint_index)
{

  const double raw_pitch = getPitch(current_pose_ptr_->pose.orientation);
  debug_values_.data.at(0) = dt;
  debug_values_.data.at(1) = current_velocity;
  debug_values_.data.at(2) = target_velocity;
  debug_values_.data.at(3) = target_acceleration;
  debug_values_.data.at(4) = double(shift);
  debug_values_.data.at(5) = pitch;
  debug_values_.data.at(6) = error_velocity;
  debug_values_.data.at(13) = raw_pitch;
  debug_values_.data.at(14) = target_velocity - current_velocity;
  debug_values_.data.at(15) = pitch * 180 / 3.141592;
  debug_values_.data.at(16) = raw_pitch * 180 / 3.141592;
  debug_values_.data.at(17) = trajectory_ptr_->points.at(closest_waypoint_index).accel.linear.x;
}

void VelocityController::writeDebugValuesCmdAcceleration(const double cmd_acceleration, const double mode)
{

  debug_values_.data.at(7) = mode;
  debug_values_.data.at(12) = cmd_acceleration;
}

void VelocityController::resetDebugValues()
{
  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);
}

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

#include "velocity_controller.h"


VelocityController::VelocityController() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_),
                                           is_sudden_stop_(false), is_smooth_stop_(false), drive_after_smooth_stop_(false),
                                           is_emergency_stop_smooth_stop_(false), prev_acceleration_(0.0)
{
  // parameters
  pnh_.param("show_debug_info", show_debug_info_, bool(false));

  // parameters for subscriber, publisher and timer
  pnh_.param("control_rate", control_rate_, double(30.0));

  // parameters to enable functions
  pnh_.param("enable_sudden_stop", enable_sudden_stop_, bool(true));
  pnh_.param("enable_smooth_stop", enable_smooth_stop_, bool(true));
  pnh_.param("enable_slope_compensation", enable_slope_compensation_, bool(true));

  // parameters to find a closest waypoint
  pnh_.param("closest_waypoint_distance_threshold", closest_dist_thr_, double(3.0));
  pnh_.param("closest_waypoint_angle_threshold", closest_angle_thr_, double(M_PI_4));

  // parameters for stop state
  pnh_.param("stop_state_velocity", stop_state_velocity_, double(0.0));                     // [m/s]
  pnh_.param("stop_state_acceleration", stop_state_acceleration_, double(-2.0));            // [m/s^2]
  pnh_.param("stop_state_entry_ego_speed", stop_state_entry_ego_speed_, double(0.2));       // [m/s]
  pnh_.param("stop_state_entry_target_speed", stop_state_entry_target_speed_, double(0.1)); // [m/s]

  // parameters for sudden stop
  pnh_.param("sudden_stop_velocity", sudden_stop_velocity_, double(0.0));          // [m/s]
  pnh_.param("sudden_stop_acceleration", sudden_stop_acceleration_, double(-2.0)); // [m/s^2]
  pnh_.param("sudden_stop_max_jerk", sudden_stop_max_jerk_, double(0.0));          // [m/s^3]
  pnh_.param("sudden_stop_min_jerk", sudden_stop_min_jerk_, double(-1.5));         // [m/s^3]

  // parameters for delay compensation
  pnh_.param("delay_compensation_time", delay_compensation_time_, double(0.17)); // [sec]

  // parameters for emergency stop by this controller
  pnh_.param("emergency_stop_acc_lim", emergency_stop_acc_lim_, double(-2.0));  // [m/s^2]
  pnh_.param("emergency_stop_jerk_lim", emergency_stop_jerk_lim_, double(1.5)); // [m/s^3]

  // parameters for smooth stop
  pnh_.param("smooth_stop/velocity_threshold_drive", velocity_threshold_drive_, double(2.2));                       // [m/s]
  pnh_.param("smooth_stop/exit_ego_speed", smooth_stop_param_.exit_ego_speed, double(2.0));                         // [m/s]
  pnh_.param("smooth_stop/exit_target_speed", smooth_stop_param_.exit_target_speed, double(2.0));                   // [m/s]
  pnh_.param("smooth_stop/entry_ego_speed", smooth_stop_param_.entry_ego_speed, double(1.0));                       // [m/s]
  pnh_.param("smooth_stop/entry_target_speed", smooth_stop_param_.entry_target_speed, double(1.0));                 // [m/s]
  pnh_.param("smooth_stop/weak_brake_time", smooth_stop_param_.weak_brake_time, double(3.0));                       // [sec]
  pnh_.param("smooth_stop/weak_brake_acc", smooth_stop_param_.weak_brake_acc, double(-0.4));                        // [m/s^2]
  pnh_.param("smooth_stop/increasing_brake_time", smooth_stop_param_.increasing_brake_time, double(3.0));           // [sec]
  pnh_.param("smooth_stop/increasing_brake_gradient", smooth_stop_param_.increasing_brake_gradient, double(-0.05)); // [m/s^3]
  pnh_.param("smooth_stop/stop_brake_time", smooth_stop_param_.stop_brake_time, double(2.0));                       // [sec]
  pnh_.param("smooth_stop/stop_brake_acc", smooth_stop_param_.stop_brake_acc, double(-1.7));                        // [m/s^2]

  // parameters for acceleration limit
  pnh_.param("max_acceleration", max_acceleration_, double(2.0));  // [m/s^2]
  pnh_.param("min_acceleration", min_acceleration_, double(-5.0)); // [m/s^2]

  // parameters for jerk limit
  pnh_.param("max_jerk", max_jerk_, double(2.0));  // [m/s^3]
  pnh_.param("min_jerk", min_jerk_, double(-5.0)); // [m/s^3]

  // parameters for slope compensation
  pnh_.param("max_pitch_rad", max_pitch_rad_, double(0.1));  // [rad]
  pnh_.param("min_pitch_rad", min_pitch_rad_, double(-0.1)); // [rad]

  pnh_.param("pid_controller/current_velocity_threshold_pid_integration", current_velocity_threshold_pid_integrate_, double(0.5)); // [m/s]

  // subscriber, publisher and timer
  sub_current_velocity_ = pnh_.subscribe("current_velocity", 1, &VelocityController::callbackCurrentVelocity, this);
  sub_trajectory_ = pnh_.subscribe("current_trajectory", 1, &VelocityController::callbackTrajectory, this);
  sub_is_sudden_stop_ = pnh_.subscribe("is_sudden_stop", 1, &VelocityController::callbackIsSuddenStop, this);
  pub_control_cmd_ = pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("control_cmd", 1);
  pub_debug_ = pnh_.advertise<std_msgs::Float32MultiArray>("debug_values", 1);
  timer_control_ = nh_.createTimer(ros::Duration(1.0 / control_rate_), &VelocityController::callbackTimerControl, this);

  // initialize PID gain
  double kp, ki, kd;
  pnh_.param("pid_controller/kp", kp, double(0.0));
  pnh_.param("pid_controller/ki", ki, double(0.0));
  pnh_.param("pid_controllerd/kd", kd, double(0.0));
  pid_velocity_.setGains(kp, ki, kd);

  // initialize PID limits
  double max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d;
  pnh_.param("pid_controller/max_out", max_pid, double(0.0));    // [m/s^2]
  pnh_.param("pid_controller/min_out", min_pid, double(0.0));    // [m/s^2]
  pnh_.param("pid_controller/max_p_effort", max_p, double(0.0)); // [m/s^2]
  pnh_.param("pid_controller/min_p_effort", min_p, double(0.0)); // [m/s^2]
  pnh_.param("pid_controller/max_i_effort", max_i, double(0.0)); // [m/s^2]
  pnh_.param("pid_controller/min_i_effort", min_i, double(0.0)); // [m/s^2]
  pnh_.param("pid_controller/max_d_effort", max_d, double(0.0)); // [m/s^2]
  pnh_.param("pid_controller/min_d_effort", min_d, double(0.0)); // [m/s^2]
  pid_velocity_.setLimits(max_pid, min_pid, max_p, min_p, max_i, min_i, max_d, min_d);

  // set lowpass filter
  double lpf_velocity_error_gain;
  pnh_.param("pid_controller/lpf_velocity_error_gain", lpf_velocity_error_gain, double(0.9));
  lpf_velocity_error.init(lpf_velocity_error_gain);

  double lpf_pitch_gain;
  pnh_.param("lpf_pitch_gain", lpf_pitch_gain, double(0.95));
  lpf_pitch.init(lpf_pitch_gain);

  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);

  /* wait to get vehicle position */
  while (ros::ok())
  {
    if (!updateCurrentPose(5.0))
    {
      ROS_INFO("[velcon] waiting map to base_link at initialize.");
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
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(timeout_sec));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN_DELAYED_THROTTLE(3.0, "[velcon] cannot get map to base_link transform. %s", ex.what());
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
  if (!updateCurrentPose(0.0))
  {
    return;
  }

  /* gurad */
  if (current_pose_ptr_ == nullptr || current_velocity_ptr_ == nullptr || trajectory_ptr_ == nullptr)
  {
    ROS_INFO_COND(show_debug_info_, "[velcon] Waiting topics, Publish stop command. pose: %d, velocity: %d, trajectory: %d",
               (int)(current_pose_ptr_ != nullptr), (int)(current_velocity_ptr_ != nullptr), (int)(trajectory_ptr_ != nullptr));
    publishCtrlCmd(stop_state_velocity_, stop_state_acceleration_, /* waiting topics */ 0);
    return;
  }

  /* initialize parameters */
  const double dt = getDt();

  /* find a closest waypoint index */
  int closest_idx;
  if (!vcutils::calcClosestWithThr(*trajectory_ptr_, current_pose_ptr_->pose, closest_angle_thr_, closest_dist_thr_, closest_idx))
  {
    ROS_ERROR_DELAYED_THROTTLE(5.0, "[velcon] calcClosestWithThr: cannot find closest! Emergency Stop! (dist_thr = %3.3f [m], angle_thr = %3.3f [rad])",
                               closest_dist_thr_, closest_angle_thr_);
    double cmd_acc = applyJerkFilter(sudden_stop_acceleration_, dt, sudden_stop_max_jerk_, sudden_stop_min_jerk_);
    publishCtrlCmd(sudden_stop_velocity_, cmd_acc, /* closest waypoint error */ 5);
    return;
  }

  double current_velocity = current_velocity_ptr_->twist.linear.x;
  double target_vel = calcInterpolatedTargetVelocity(*trajectory_ptr_, *current_pose_ptr_, current_velocity, closest_idx);
  double target_acc = DelayCompensator::computeCommandAccelerationAfterDelayTime(*trajectory_ptr_, closest_idx, delay_compensation_time_,
                                                                                 current_velocity_ptr_->twist.linear.x);

  /* shift check */
  const Shift shift = getCurrentShiftMode(target_vel, target_acc);
  if (shift != prev_shift_)
  {
    pid_velocity_.reset();
  }
  prev_shift_ = shift;

  const double pitch = lpf_pitch.filter(getPitch(current_pose_ptr_->pose.orientation));
  const double error_velocity = lpf_velocity_error.filter(target_vel - current_velocity);
  ROS_INFO_COND(show_debug_info_, "[velcon] current_vel = %f, target_vel = %f, target_acc = %f, pitch = %f, error_velocity = %f",
                current_velocity, target_vel, target_acc, pitch, error_velocity);
  writeDebugValues(dt, current_velocity, target_vel, target_acc, shift, pitch, error_velocity, closest_idx);

  // sudden stop
  if (enable_sudden_stop_ && is_sudden_stop_)
  {
    double cmd_acc = applyJerkFilter(sudden_stop_acceleration_, dt, sudden_stop_max_jerk_, sudden_stop_min_jerk_);
    publishCtrlCmd(sudden_stop_velocity_, cmd_acc, /* suden stop */ 3);
    pid_velocity_.reset();
    resetSmoothStop();

    ROS_INFO_COND(show_debug_info_, "[velcon] STOP!");
    return;
  }

  // stop state
  if (std::fabs(current_velocity) < stop_state_entry_ego_speed_ &&
      std::fabs(target_vel) < stop_state_entry_target_speed_ && !is_smooth_stop_)
  {
    double cmd_acc = applyAccFilter(stop_state_acceleration_, max_acceleration_, min_acceleration_);
    cmd_acc = applyJerkFilter(cmd_acc, dt, max_jerk_, min_jerk_);
    cmd_acc = applySlopeCompensation(cmd_acc, pitch, shift);
    publishCtrlCmd(stop_state_velocity_, cmd_acc, /* stopping */ 2);
    pid_velocity_.reset();
    resetSmoothStop();
    resetEmergencyStop();

    ROS_INFO_COND(show_debug_info_, "[velcon] Stopping");
    return;
  }

  // emergency stop by this controller
  if (is_emergency_stop_smooth_stop_)
  {
    double cmd_acc = applyJerkFilter(emergency_stop_acc_lim_, dt, std::fabs(emergency_stop_jerk_lim_), emergency_stop_jerk_lim_);
    publishCtrlCmd(0.0, cmd_acc, /* emergency stop */ 6);
    pid_velocity_.reset();
    resetSmoothStop();

    ROS_WARN_DELAYED_THROTTLE(3.0, "[velcon] Emergency stop!!!");
    return;
  }

  // smooth stop
  if (enable_smooth_stop_)
  {
    ROS_INFO_COND(show_debug_info_, "[velcon] smooth stop: is_smooth_stop = %d, drive_after_stop = %d", is_smooth_stop_, drive_after_smooth_stop_);
    // If the current or target velocity increases during smooth stopping, exit smooth stopping
    if (is_smooth_stop_ && (std::fabs(current_velocity) > smooth_stop_param_.exit_ego_speed))
    {
      is_smooth_stop_ = false;
      is_emergency_stop_smooth_stop_ = true;
    }

    if (is_smooth_stop_ && (std::fabs(target_vel) > smooth_stop_param_.exit_target_speed))
    {
      is_smooth_stop_ = false;
    }

    if (std::fabs(current_velocity) < smooth_stop_param_.entry_ego_speed &&
        std::fabs(target_vel) < smooth_stop_param_.entry_target_speed && drive_after_smooth_stop_)
    {
      is_smooth_stop_ = true;
      drive_after_smooth_stop_ = false;
      start_time_smooth_stop_ = ros::Time::now();
    }

    if (is_smooth_stop_)
    {
      double cmd_acc = calcSmoothStopAcc();
      cmd_acc = applySlopeCompensation(cmd_acc, pitch, shift);
      cmd_acc = applyAccFilter(cmd_acc, max_acceleration_, min_acceleration_);
      cmd_acc = applyJerkFilter(cmd_acc, dt, max_jerk_, min_jerk_);
      publishCtrlCmd(target_vel, cmd_acc, /* smooth stop */ 4);
      pid_velocity_.reset();
      ROS_INFO_COND(show_debug_info_, "[velcon] smooth stop: Smooth stopping. cmd_acc = %f", cmd_acc);
      return;
    }

    if (std::fabs(current_velocity) > velocity_threshold_drive_)
    {
      drive_after_smooth_stop_ = true;
    }
  }

  // velocity feedback
  ROS_INFO_COND(show_debug_info_, "[velcon] feedback control: current_vel = %f, target_vel = %f, target_acc = %f", current_velocity, target_vel, target_acc);
  const bool enable_integration = std::fabs(current_velocity) < current_velocity_threshold_pid_integrate_ ? false : true;

  double cmd_acc = applyVelocityFeedback(target_acc, error_velocity, dt, enable_integration);
  cmd_acc = applySlopeCompensation(cmd_acc, pitch, shift);
  cmd_acc = applyAccFilter(cmd_acc, max_acceleration_, min_acceleration_);
  cmd_acc = applyJerkFilter(cmd_acc, dt, max_jerk_, min_jerk_);

  ROS_INFO_COND(show_debug_info_, "[velcon] FB: after PID and filters cmd_acc = %f", cmd_acc);
  publishCtrlCmd(target_vel, cmd_acc, /* PIC control */ 1);
  return;
}

void VelocityController::publishCtrlCmd(const double velocity, const double acceleration, const int controller_mode)
{
  prev_acceleration_ = acceleration;

  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "base_link";
  cmd.control.velocity = velocity;
  cmd.control.acceleration = acceleration;
  pub_control_cmd_.publish(cmd);

  // debug
  debug_values_.data.at(7) = (double)controller_mode;
  debug_values_.data.at(12) = acceleration;
  pub_debug_.publish(debug_values_);
  debug_values_.data.clear();
  debug_values_.data.resize(num_debug_values_, 0.0);
}

double VelocityController::getDt()
{
  double dt;
  if (!prev_control_time_)
  {
    dt = 1.0 / control_rate_;
    prev_control_time_ = std::make_shared<ros::Time>(ros::Time::now());
  }
  else
  {
    dt = (ros::Time::now() - *prev_control_time_).toSec();
    *prev_control_time_ = ros::Time::now();
  }
  const double max_dt = 1.0 / control_rate_ * 2.0;
  const double min_dt = 1.0 / control_rate_ * 0.5;
  return std::max(std::min(dt, max_dt), min_dt);
}

enum VelocityController::Shift VelocityController::getCurrentShiftMode(const double target_vel,
                                                                       const double target_acc)
{
  const double ep = 1.0e-5;
  if (target_vel > ep)
  {
    return Shift::Forward;
  }
  else if (target_vel < -ep)
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

double VelocityController::calcSmoothStopAcc()
{
  const double elapsed_time = (ros::Time::now() - start_time_smooth_stop_).toSec();

  double cmd_acc;
  if (elapsed_time < smooth_stop_param_.weak_brake_time)
  {
    cmd_acc = smooth_stop_param_.weak_brake_acc;
    ROS_INFO_COND(show_debug_info_, "[VC Smooth Stop] weak breaking! acc = %f", cmd_acc);
  }
  else if (elapsed_time < (smooth_stop_param_.weak_brake_time + smooth_stop_param_.increasing_brake_time))
  {
    const double dt = elapsed_time - smooth_stop_param_.weak_brake_time;
    cmd_acc = smooth_stop_param_.weak_brake_acc + smooth_stop_param_.increasing_brake_gradient * dt;
    ROS_INFO_COND(show_debug_info_, "[VC Smooth Stop] break increasing! acc = %f", cmd_acc);
  }
  else if (elapsed_time < (smooth_stop_param_.weak_brake_time + smooth_stop_param_.increasing_brake_time + smooth_stop_param_.stop_brake_time))
  {
    cmd_acc = smooth_stop_param_.stop_brake_acc;
    ROS_INFO_COND(show_debug_info_, "[VC Smooth Stop] stop breaking! acc = %f", cmd_acc);
  }
  else
  {
    is_smooth_stop_ = false;
    cmd_acc = smooth_stop_param_.stop_brake_acc;
    ROS_INFO_COND(show_debug_info_, "[VC Smooth Stop] finish smooth stopping! acc = %f", cmd_acc);
  }

  return cmd_acc;
}

double VelocityController::calcInterpolatedTargetVelocity(const autoware_planning_msgs::Trajectory &traj,
                                                          const geometry_msgs::PoseStamped &curr_pose, const double curr_vel, const int closest)
{
  const double closest_vel = traj.points.at(closest).twist.linear.x;

  if (traj.points.size() < 2)
  {
    return closest_vel;
  }

  /* If the current position is at the edge of the reference trajectory, enable the edge velocity. Else, calc secondary closest index for interpolation */
  int closest_second;
  geometry_msgs::Point rel_pos = vcutils::transformToRelativeCoordinate2D(curr_pose.pose.position, traj.points.at(closest).pose);
  if (closest == 0)
  {
    if (rel_pos.x * curr_vel <= 0.0)
    {
      return closest_vel;
    }
    closest_second = 1;
  }
  else if (closest == traj.points.size() - 1)
  {
    if (rel_pos.x * curr_vel >= 0.0)
    {
      return closest_vel;
    }
    closest_second = traj.points.size() - 2;
  }
  else
  {
    const double dist1 = vcutils::calcDistSquared2D(traj.points.at(closest).pose, traj.points.at(closest - 1).pose);
    const double dist2 = vcutils::calcDistSquared2D(traj.points.at(closest).pose, traj.points.at(closest + 1).pose);
    closest_second = dist1 < dist2 ? closest - 1 : closest + 1;
  }

  /* apply linear interpolation */
  const double dist_c1 = std::sqrt(vcutils::calcDistSquared2D(curr_pose.pose, traj.points.at(closest).pose));
  const double dist_c2 = std::sqrt(vcutils::calcDistSquared2D(curr_pose.pose, traj.points.at(closest_second).pose));
  const double v1 = traj.points.at(closest).twist.linear.x;
  const double v2 = traj.points.at(closest_second).twist.linear.x;
  const double vel_interp = (dist_c1 * v2 + dist_c2 * v1) / (dist_c1 + dist_c2);

  return vel_interp;
}

double VelocityController::applyAccFilter(const double acceleration, const double max_acceleration,
                                          const double min_acceleration)
{
  const double filtered_acceleration = std::min(std::max(acceleration, min_acceleration), max_acceleration);
  debug_values_.data.at(9) = filtered_acceleration;
  return filtered_acceleration;
}

double VelocityController::applyJerkFilter(const double acceleration, const double dt, const double max_jerk,
                                           const double min_jerk)
{
  const double jerk = (acceleration - prev_acceleration_) / dt;
  const double filtered_jerk = std::min(std::max(jerk, min_jerk), max_jerk);
  const double filtered_acceleration = prev_acceleration_ + (filtered_jerk * dt);
  debug_values_.data.at(10) = filtered_acceleration;
  return filtered_acceleration;
}

double VelocityController::applySlopeCompensation(const double acceleration, const double pitch,
                                                  const Shift shift)
{
  if (!enable_slope_compensation_)
  {
    return acceleration;
  }
  const double gravity = 9.80665;
  const double pitch_limited = std::min(std::max(pitch, min_pitch_rad_), max_pitch_rad_);
  double sign = (shift == Shift::Forward) ? -1 : (shift == Shift::Reverse ? 1 : 0);
  double compensated_acceleration = acceleration + sign * gravity * std::sin(pitch_limited);
  debug_values_.data.at(11) = compensated_acceleration;
  return compensated_acceleration;

}

double VelocityController::applyVelocityFeedback(const double target_acc, const double error_velocity,
                                                 const double dt, const bool enable_integration)
{
  std::vector<double> pid_contributions(3);
  double feedbacked_acc = target_acc + pid_velocity_.calculate(error_velocity, dt,
                                                               enable_integration, pid_contributions);
  debug_values_.data.at(8) = feedbacked_acc;
  debug_values_.data.at(18) = pid_contributions.at(0);
  debug_values_.data.at(19) = pid_contributions.at(1);
  debug_values_.data.at(20) = pid_contributions.at(2);
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

void VelocityController::writeDebugValues(const double dt, const double current_vel, const double target_vel, const double target_acc,
                                          const Shift shift, const double pitch, const double error_vel, const int32_t closest)
{
  const double rad2deg = 180.0 / 3.141592;
  const double raw_pitch = getPitch(current_pose_ptr_->pose.orientation);
  debug_values_.data.at(0) = dt;
  debug_values_.data.at(1) = current_vel;
  debug_values_.data.at(2) = target_vel;
  debug_values_.data.at(3) = target_acc;
  debug_values_.data.at(4) = double(shift);
  debug_values_.data.at(5) = pitch;
  debug_values_.data.at(6) = error_vel;
  debug_values_.data.at(13) = raw_pitch;
  debug_values_.data.at(14) = target_vel - current_vel;
  debug_values_.data.at(15) = pitch * rad2deg;
  debug_values_.data.at(16) = raw_pitch * rad2deg;
  debug_values_.data.at(17) = trajectory_ptr_->points.at(closest).accel.linear.x;
}

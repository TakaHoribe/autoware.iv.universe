/*
 * Copyright 2017-2019 Autoware Foundation. All rights reserved.
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

#include "pacmod_interface/pacmod_interface.h"

PacmodInterface::PacmodInterface()
  : nh_()
  , private_nh_("~")
  , engage_(false)
  , prev_engage_(false)
  , vehicle_cmd_initialized_(false)
  , wheel_speed_initialized_(false)
  , steer_wheel_initialized_(false)
{
  // setup parameters
  private_nh_.param<std::string>("base_frame_id", base_frame_id_, "base_link");
  private_nh_.param<int>("command_timeout", command_timeout_, int(1000));
  private_nh_.param<double>("loop_rate", loop_rate_, double(30.0));

  // parameters for vehicle specifications
  private_nh_.param<double>("/vehicle_info/wheel_radius", tire_radius_, double(0.39));
  private_nh_.param<double>("/vehicle_info/wheel_base", wheel_base_, double(2.79));
  private_nh_.param<double>("agr_coef_a", agr_coef_a_, double(15.713));
  private_nh_.param<double>("agr_coef_b", agr_coef_b_, double(0.053));
  private_nh_.param<double>("agr_coef_c", agr_coef_c_, double(0.042));

  // parameters for accel/brake map
  std::string home_dir{ std::getenv("HOME") };
  private_nh_.param<std::string>("csv_path_accel_map", csv_path_accel_map_,
                                 home_dir += "/.autoware/vehicle_data/lexus/accel_map.csv");
  private_nh_.param<std::string>("csv_path_brake_map", csv_path_brake_map_,
                                 home_dir += "/.autoware/vehicle_data/lexus/brake_map.csv");

  // parameters for emergency stop
  private_nh_.param<double>("acc_emergency", acc_emergency_, double(-2.0));

  // parameters for limitter
  private_nh_.param<double>("max_throttle", max_throttle_, double(0.2));
  private_nh_.param<double>("min_throttle", min_throttle_, double(0.0));
  private_nh_.param<double>("max_brake", max_brake_, double(0.8));
  private_nh_.param<double>("min_brake", min_brake_, double(0.0));
  private_nh_.param<double>("max_steering_wheel", max_steering_wheel_, double(2.7 * M_PI));
  private_nh_.param<double>("min_steering_wheel", min_steering_wheel_, double(-2.7 * M_PI));

  rate_ = new ros::Rate(loop_rate_);

  // subscribers
  vehicle_cmd_sub_ = nh_.subscribe("/control/vehicle_cmd", 1, &PacmodInterface::callbackVehicleCmd, this);
  engage_sub_ = nh_.subscribe("vehicle/engage", 1, &PacmodInterface::callbackEngage, this);
  steer_wheel_sub_ = nh_.subscribe("pacmod/parsed_tx/steer_rpt", 1, &PacmodInterface::callbackSteerWheel, this);
  wheel_speed_sub_ = nh_.subscribe("pacmod/parsed_tx/wheel_speed_rpt", 1, &PacmodInterface::callbackWheelSpeed, this);

  // publisher
  accel_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdFloat>("pacmod/as_rx/accel_cmd", 10);
  brake_cmd_pub_ = nh_.advertise<pacmod_msgs::SystemCmdFloat>("pacmod/as_rx/brake_cmd", 10);
  steer_cmd_pub_ = nh_.advertise<pacmod_msgs::SteerSystemCmd>("pacmod/as_rx/steer_cmd", 10);
  vehicle_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("vehicle/velocity", 10);
}

PacmodInterface::~PacmodInterface()
{
}

void PacmodInterface::run()
{
  if (!accel_map_.readAccelMapFromCSV(csv_path_accel_map_))
  {
    ROS_ERROR("Cannot read %s", csv_path_accel_map_.c_str());
    return;
  }
  if (!brake_map_.readBrakeMapFromCSV(csv_path_brake_map_))
  {
    ROS_ERROR("Cannot read %s", csv_path_brake_map_.c_str());
    return;
  }

  while (ros::ok())
  {
    ros::spinOnce();
    publishCommand();
    rate_->sleep();
  }
}

void PacmodInterface::callbackVehicleCmd(const autoware_control_msgs::VehicleCommandStamped::ConstPtr& msg)
{
  command_time_ = ros::Time::now();
  vehicle_cmd_ptr_ = std::make_shared<autoware_control_msgs::VehicleCommandStamped>(*msg);
  vehicle_cmd_initialized_ = true;
}

void PacmodInterface::callbackEngage(const std_msgs::BoolConstPtr& msg)
{
  engage_ = msg->data;
}

void PacmodInterface::callbackSteerWheel(const pacmod_msgs::SystemRptFloatConstPtr& msg)
{
  steer_wheel_ = msg->output;
  steer_wheel_initialized_ = true;
}

void PacmodInterface::callbackWheelSpeed(const pacmod_msgs::WheelSpeedRptConstPtr& msg)
{
  wheel_speed_ =
      (msg->rear_left_wheel_speed + msg->rear_right_wheel_speed) / 2 * tire_radius_;  // [rad/s] * [m] = [m/s]
  wheel_speed_initialized_ = true;
}

void PacmodInterface::publishCommand()
{
  if (!(vehicle_cmd_initialized_ && wheel_speed_initialized_ && steer_wheel_initialized_))
  {
    return;
  }

  double desired_acc = vehicle_cmd_ptr_->command.control.acceleration;

  ros::Time stamp = ros::Time::now();
  bool emergency = (vehicle_cmd_.command.emergency == 1);
  bool timeouted = (((ros::Time::now() - command_time_).toSec() * 1000) > command_timeout_);
  if (emergency || timeouted)
  {
    ROS_ERROR("[pacmod interface] Emergency Stopping, emergency = %d, timeouted = %d", emergency, timeouted);
    desired_acc = acc_emergency_;
  }

  double desired_throttle = 0;
  double desired_brake = 0;

  ROS_INFO("wheel_speed_ = %f, desired_acc = %f", wheel_speed_, desired_acc);
  // throttle mode
  if (!accel_map_.getThrottle(desired_acc, std::abs(wheel_speed_), desired_throttle))
  {
    // brake mode
    brake_map_.getBrake(desired_acc, std::abs(wheel_speed_), desired_brake);
  }
  desired_throttle = std::min(std::max(desired_throttle, min_throttle_), max_throttle_);
  desired_brake = std::min(std::max(desired_brake, min_brake_), max_brake_);

  bool flag_clear = false;
  if (!prev_engage_ && engage_)
  {
    flag_clear = true;
  }
  prev_engage_ = engage_;

  pacmod_msgs::SystemCmdFloat accel_cmd;
  accel_cmd.header.frame_id = base_frame_id_;
  accel_cmd.header.stamp = stamp;
  accel_cmd.enable = engage_;
  accel_cmd.ignore_overrides = false;
  accel_cmd.clear_override = flag_clear;
  accel_cmd.clear_faults = flag_clear;
  accel_cmd.command = desired_throttle;
  accel_cmd_pub_.publish(accel_cmd);

  pacmod_msgs::SystemCmdFloat brake_cmd;
  brake_cmd.header.frame_id = base_frame_id_;
  brake_cmd.header.stamp = stamp;
  brake_cmd.enable = engage_;
  brake_cmd.ignore_overrides = false;
  brake_cmd.clear_override = flag_clear;
  brake_cmd.clear_faults = flag_clear;
  brake_cmd.command = desired_brake;
  brake_cmd_pub_.publish(brake_cmd);

  double adaptive_gear_ratio =
      std::max(1e-5, agr_coef_a_ + agr_coef_b_ * wheel_speed_ * wheel_speed_ - agr_coef_c_ * steer_wheel_);
  double desired_steer = vehicle_cmd_ptr_->command.control.steering_angle;
  double desired_steer_wheel = desired_steer * adaptive_gear_ratio;
  desired_steer_wheel = std::min(std::max(desired_steer_wheel, min_steering_wheel_), max_steering_wheel_);

  double desired_rotation_rate = 6.6;

  pacmod_msgs::SteerSystemCmd steer_cmd;
  steer_cmd.header.frame_id = base_frame_id_;
  steer_cmd.header.stamp = stamp;
  steer_cmd.enable = engage_;
  steer_cmd.ignore_overrides = false;
  steer_cmd.clear_override = flag_clear;
  steer_cmd.clear_faults = flag_clear;
  steer_cmd.command = desired_steer_wheel;
  steer_cmd.rotation_rate = desired_rotation_rate;
  steer_cmd_pub_.publish(steer_cmd);

  double curr_curvature = std::tan(steer_wheel_ / adaptive_gear_ratio) / wheel_base_;

  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = base_frame_id_;
  twist.header.stamp = stamp;
  twist.twist.linear.x = wheel_speed_;                    // [m/s]
  twist.twist.angular.z = wheel_speed_ * curr_curvature;  // [rad/s]
  vehicle_twist_pub_.publish(twist);
}

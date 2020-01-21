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

#include "ssc_interface/ssc_interface.h"

SSCInterface::SSCInterface() : nh_(), private_nh_("~"), engage_(false), command_initialized_(false)
{
  // setup parameters
  private_nh_.param<bool>("use_rear_wheel_speed", use_rear_wheel_speed_, true);
  private_nh_.param<bool>("use_adaptive_gear_ratio", use_adaptive_gear_ratio_, true);
  private_nh_.param<int>("command_timeout", command_timeout_, 1000);
  private_nh_.param<double>("loop_rate", loop_rate_, 30.0);
  private_nh_.param<double>("/vehicle_info/wheel_base", wheel_base_, 2.79);
  private_nh_.param<double>("/vehicle_info/wheel_radius", tire_radius_, 0.39);
  private_nh_.param<double>("ssc_gear_ratio", ssc_gear_ratio_, 16.135);
  private_nh_.param<double>("acceleration_limit", acceleration_limit_, 3.0);
  private_nh_.param<double>("deceleration_limit", deceleration_limit_, 3.0);
  private_nh_.param<double>("max_curvature_rate", max_curvature_rate_, 0.15);
  private_nh_.param<double>("agr_coef_a", agr_coef_a_, 15.713);
  private_nh_.param<double>("agr_coef_b", agr_coef_b_, 0.053);
  private_nh_.param<double>("agr_coef_c", agr_coef_c_, 0.042);
  private_nh_.param<double>("steering_offset", steering_offset_, 0.0);

  rate_ = new ros::Rate(loop_rate_);

  // subscribers from autoware
  vehicle_cmd_sub_ = nh_.subscribe("/control/vehicle_cmd", 1, &SSCInterface::callbackFromVehicleCmd, this);
  engage_sub_ = nh_.subscribe("vehicle/engage", 1, &SSCInterface::callbackFromEngage, this);

  // subscribers from SSC
  module_states_sub_ = nh_.subscribe("as/module_states", 1, &SSCInterface::callbackFromSSCModuleStates, this);
  curvature_feedback_sub_ =
      new message_filters::Subscriber<automotive_platform_msgs::CurvatureFeedback>(nh_, "as/curvature_feedback", 10);
  throttle_feedback_sub_ =
      new message_filters::Subscriber<automotive_platform_msgs::ThrottleFeedback>(nh_, "as/throttle_feedback", 10);
  brake_feedback_sub_ =
      new message_filters::Subscriber<automotive_platform_msgs::BrakeFeedback>(nh_, "as/brake_feedback", 10);
  gear_feedback_sub_ =
      new message_filters::Subscriber<automotive_platform_msgs::GearFeedback>(nh_, "as/gear_feedback", 10);
  velocity_accel_sub_ =
      new message_filters::Subscriber<automotive_platform_msgs::VelocityAccel>(nh_, "as/velocity_accel", 10);
  // subscribers from PACMod
  wheel_speed_sub_ =
      new message_filters::Subscriber<pacmod_msgs::WheelSpeedRpt>(nh_, "pacmod/parsed_tx/wheel_speed_rpt", 10);
  steering_wheel_sub_ =
      new message_filters::Subscriber<pacmod_msgs::SystemRptFloat>(nh_, "pacmod/parsed_tx/steer_rpt", 10);
  ssc_feedbacks_sync_ = new message_filters::Synchronizer<SSCFeedbacksSyncPolicy>(
      SSCFeedbacksSyncPolicy(10), *velocity_accel_sub_, *curvature_feedback_sub_, *throttle_feedback_sub_,
      *brake_feedback_sub_, *gear_feedback_sub_, *wheel_speed_sub_, *steering_wheel_sub_);
  ssc_feedbacks_sync_->registerCallback(
      boost::bind(&SSCInterface::callbackFromSSCFeedbacks, this, _1, _2, _3, _4, _5, _6, _7));

  // publishers to autoware
  vehicle_status_pub_ = nh_.advertise<autoware_control_msgs::VehicleStatusStamped>("/vehicle/status", 10);
  current_twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/vehicle/status/twist", 10);
  current_steer_pub_ = nh_.advertise<std_msgs::Float32>("/vehicle/status/steering", 10);
  current_steer_wheel_deg_pub_ = nh_.advertise<std_msgs::Float32>("/vehicle/status/steering_wheel_deg", 10);
  current_velocity_pub_ = nh_.advertise<std_msgs::Float32>("/vehicle/status/velocity", 10);
  current_velocity_kmph_pub_ = nh_.advertise<std_msgs::Float32>("/vehicle/status/velocity_kmph", 10);

  // publishers to SSC
  steer_mode_pub_ = nh_.advertise<automotive_platform_msgs::SteerMode>("as/arbitrated_steering_commands", 10);
  speed_mode_pub_ = nh_.advertise<automotive_platform_msgs::SpeedMode>("as/arbitrated_speed_commands", 10);
  turn_signal_pub_ = nh_.advertise<automotive_platform_msgs::TurnSignalCommand>("as/turn_signal_command", 10);
  gear_pub_ = nh_.advertise<automotive_platform_msgs::GearCommand>("as/gear_select", 1, true);
}

SSCInterface::~SSCInterface()
{
}

void SSCInterface::run()
{
  while (ros::ok())
  {
    ros::spinOnce();
    publishCommand();
    rate_->sleep();
  }
}

void SSCInterface::callbackFromVehicleCmd(const autoware_control_msgs::VehicleCommandStampedConstPtr& msg)
{
  command_time_ = ros::Time::now();
  vehicle_cmd_ = *msg;
  command_initialized_ = true;
}

void SSCInterface::callbackFromEngage(const std_msgs::BoolConstPtr& msg)
{
  engage_ = msg->data;
}

void SSCInterface::callbackFromSSCModuleStates(const automotive_navigation_msgs::ModuleStateConstPtr& msg)
{
  if (msg->name.find("veh_controller") != std::string::npos)
  {
    module_states_ = *msg;  // *_veh_controller status is used for 'drive/steeringmode'
  }
}

void SSCInterface::callbackFromSSCFeedbacks(const automotive_platform_msgs::VelocityAccelConstPtr& msg_velocity,
                                            const automotive_platform_msgs::CurvatureFeedbackConstPtr& msg_curvature,
                                            const automotive_platform_msgs::ThrottleFeedbackConstPtr& msg_throttle,
                                            const automotive_platform_msgs::BrakeFeedbackConstPtr& msg_brake,
                                            const automotive_platform_msgs::GearFeedbackConstPtr& msg_gear,
                                            const pacmod_msgs::WheelSpeedRptConstPtr& msg_wheel_speed,
                                            const pacmod_msgs::SystemRptFloatConstPtr& msg_steering_wheel)
{
  ros::Time stamp = msg_velocity->header.stamp;
  // ros::Time stamp = msg_wheel_speed->header.stamp;

  // current speed
  double speed =
      !use_rear_wheel_speed_ ?
          (msg_velocity->velocity) :
          (msg_wheel_speed->rear_left_wheel_speed + msg_wheel_speed->rear_right_wheel_speed) * tire_radius_ / 2.;

  // update adaptive gear ratio (avoiding zero divizion)
  adaptive_gear_ratio_ =
    std::max(1e-5, agr_coef_a_ + agr_coef_b_ * speed * speed - agr_coef_c_ * msg_steering_wheel->output);
  // current steering curvature
  double curvature = !use_adaptive_gear_ratio_ ?
                         (msg_curvature->curvature) :
                         std::tan(msg_steering_wheel->output / adaptive_gear_ratio_ - steering_offset_) / wheel_base_;

  // constexpr double tread = 1.64;  // spec sheet 1.63
  // double omega =
  //   (-msg_wheel_speed->rear_right_wheel_speed + msg_wheel_speed->rear_left_wheel_speed) * tire_radius_ / tread;

  // as_current_velocity (geometry_msgs::TwistStamped)
  geometry_msgs::TwistStamped twist;
  twist.header.frame_id = BASE_FRAME_ID;
  twist.header.stamp = stamp;
  twist.twist.linear.x = speed;               // [m/s]
  twist.twist.angular.z = curvature * speed;  // [rad/s]
  // twist.twist.angular.z = omega;  // [rad/s]
  current_twist_pub_.publish(twist);

  // vehicle_status (autoware_control_msgs::VehicleStatus)
  autoware_control_msgs::VehicleStatusStamped vehicle_status;
  vehicle_status.header.frame_id = BASE_FRAME_ID;
  vehicle_status.header.stamp = stamp;

  // drive/steeringmode
  vehicle_status.status.mode.mode = (module_states_.state == "active") ? autoware_control_msgs::ControlMode::AUTO :
                                                                  autoware_control_msgs::ControlMode::MANUAL;

  // speed [km/h]
  vehicle_status.status.velocity = speed;

  // drive/brake pedal [0,1000] (TODO: Scaling)
  // vehicle_status.drivepedal = (int)(1000 * msg_throttle->throttle_pedal);
  // vehicle_status.brakepedal = (int)(1000 * msg_brake->brake_pedal);

  // steering angle [rad]
  vehicle_status.status.steering_angle = std::atan(curvature * wheel_base_);

  // gearshift
  if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::NONE)
  {
    vehicle_status.status.shift.shift = -1;
  }
  else if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::PARK)
  {
    vehicle_status.status.shift.shift = autoware_control_msgs::Shift::PARKING;
  }
  else if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::REVERSE)
  {
    vehicle_status.status.shift.shift = autoware_control_msgs::Shift::REVERSE;
  }
  else if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::NEUTRAL)
  {
    vehicle_status.status.shift.shift = autoware_control_msgs::Shift::NEUTRAL;
  }
  else if (msg_gear->current_gear.gear == automotive_platform_msgs::Gear::DRIVE)
  {
    vehicle_status.status.shift.shift = autoware_control_msgs::Shift::DRIVE;
  }

  // lamp/light cannot be obtain from SSC
  // vehicle_status.lamp
  // vehicle_status.light

  vehicle_status_pub_.publish(vehicle_status);

  std_msgs::Float32 vel;
  vel.data = twist.twist.linear.x;
  current_velocity_pub_.publish(vel);

  std_msgs::Float32 vel_kmph;
  vel_kmph.data = twist.twist.linear.x / 3.6;
  current_velocity_kmph_pub_.publish(vel_kmph);

  std_msgs::Float32 steer;
  steer.data = vehicle_status.status.steering_angle - steering_offset_;
  current_steer_pub_.publish(steer);

  std_msgs::Float32 steer_wheel_deg;
  steer_wheel_deg.data = msg_steering_wheel->output;
  current_steer_wheel_deg_pub_.publish(steer_wheel_deg);
}

void SSCInterface::publishCommand()
{
  if (!command_initialized_)
  {
    return;
  }

  ros::Time stamp = ros::Time::now();

  // Desired values
  // Driving mode (If autonomy mode should be active, mode = 1)
  unsigned char desired_mode = engage_ ? 1 : 0;

  // Speed for SSC speed_model
  double desired_speed = vehicle_cmd_.command.control.velocity;

  // Curvature for SSC steer_model
  double desired_steering_angle = !use_adaptive_gear_ratio_ ?
                                      vehicle_cmd_.command.control.steering_angle + steering_offset_ :
                                      (vehicle_cmd_.command.control.steering_angle + steering_offset_) * ssc_gear_ratio_ / adaptive_gear_ratio_;
  double desired_curvature = std::tan(desired_steering_angle) / wheel_base_;

  // Gear (TODO: Use vehicle_cmd.gear)
  unsigned char desired_shift = engage_ ? automotive_platform_msgs::Gear::DRIVE : automotive_platform_msgs::Gear::NONE;

  // Turn signal
  unsigned char desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;

  if (vehicle_cmd_.command.turn_signal.signal == autoware_control_msgs::TurnSignal::NONE)
  {
    desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::NONE;
  }
  else if (vehicle_cmd_.command.turn_signal.signal == autoware_control_msgs::TurnSignal::LEFT)
  {
    desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::LEFT;
  }
  else if (vehicle_cmd_.command.turn_signal.signal == autoware_control_msgs::TurnSignal::RIGHT)
  {
    desired_turn_signal = automotive_platform_msgs::TurnSignalCommand::RIGHT;
  }
  else if (vehicle_cmd_.command.turn_signal.signal == autoware_control_msgs::TurnSignal::HAZARD)
  {
    // NOTE: HAZARD signal cannot be used in automotive_platform_msgs::TurnSignalCommand
  }

  // Override desired speed to ZERO by emergency/timeout
  bool emergency = (vehicle_cmd_.command.emergency == 1);
  bool timeouted = (((ros::Time::now() - command_time_).toSec() * 1000) > command_timeout_);

  if (emergency || timeouted)
  {
    ROS_ERROR("Emergency Stopping, emergency = %d, timeouted = %d", emergency, timeouted);
    desired_speed = 0.0;
  }

  // speed command
  automotive_platform_msgs::SpeedMode speed_mode;
  speed_mode.header.frame_id = BASE_FRAME_ID;
  speed_mode.header.stamp = stamp;
  speed_mode.mode = desired_mode;
  speed_mode.speed = desired_speed;
  speed_mode.acceleration_limit = acceleration_limit_;
  speed_mode.deceleration_limit = deceleration_limit_;

  // steer command
  automotive_platform_msgs::SteerMode steer_mode;
  steer_mode.header.frame_id = BASE_FRAME_ID;
  steer_mode.header.stamp = stamp;
  steer_mode.mode = desired_mode;
  steer_mode.curvature = desired_curvature;
  steer_mode.max_curvature_rate = max_curvature_rate_;

  // turn signal command
  automotive_platform_msgs::TurnSignalCommand turn_signal;
  turn_signal.header.frame_id = BASE_FRAME_ID;
  turn_signal.header.stamp = stamp;
  turn_signal.mode = desired_mode;
  turn_signal.turn_signal = desired_turn_signal;

  // gear_cmd command
  automotive_platform_msgs::GearCommand gear_cmd;
  gear_cmd.header.frame_id = BASE_FRAME_ID;
  gear_cmd.header.stamp = stamp;
  gear_cmd.command.gear = desired_shift;

  // publish
  speed_mode_pub_.publish(speed_mode);
  steer_mode_pub_.publish(steer_mode);
  turn_signal_pub_.publish(turn_signal);
  gear_pub_.publish(gear_cmd);

  ROS_INFO_STREAM("Mode: " << (int)desired_mode << ", "
                           << "Speed: " << speed_mode.speed << ", "
                           << "Curvature: " << steer_mode.curvature << ", "
                           << "Gear: " << (int)gear_cmd.command.gear << ", "
                           << "TurnSignal: " << (int)turn_signal.turn_signal);
}

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

#include <algorithm>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_control_msgs/GateMode.h>
#include <autoware_vehicle_msgs/RawControlCommandStamped.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/TurnSignal.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>

class JoyConverter
{
public:
  explicit JoyConverter(const sensor_msgs::Joy & j) : j_(j) {}

  const float accel() const
  {
    const auto button = static_cast<float>(j_.buttons.at(0));  // Cross
    const auto stick = std::max(0.0f, j_.axes.at(4));          // R Stick
    const auto trigger = std::max(0.0f, -j_.axes.at(5));       // R Trigger
    return std::max({button, stick, trigger});
  }

  const float brake() const
  {
    const auto button = static_cast<float>(j_.buttons.at(3));  // Square
    const auto stick = std::max(0.0f, -j_.axes.at(4));         // R Stick
    const auto trigger = std::max(0.0f, -j_.axes.at(2));       // L Trigger
    return std::max({button, stick, trigger});
  }

  const float steer() const { return j_.axes.at(0); }                                   // L Stick
  const bool shift_up() const { return static_cast<bool>(j_.axes.at(7) == 1); }         // Up
  const bool shift_down() const { return static_cast<bool>(j_.axes.at(7) == -1); }      // Down
  const bool turn_signal_left() const { return static_cast<bool>(j_.buttons.at(4)); }   // L1
  const bool turn_signal_right() const { return static_cast<bool>(j_.buttons.at(5)); }  // R1
  const bool gate_mode() const { return static_cast<bool>(j_.buttons.at(1)); }          // Circle
  const bool emergency() const { return static_cast<bool>(j_.buttons.at(2)); }          // Triangle
  const bool engage() const { return static_cast<bool>(j_.buttons.at(9)); }             // Options
  const bool disengage() const { return static_cast<bool>(j_.buttons.at(8)); }          // Share

private:
  const sensor_msgs::Joy j_;
};

using ShiftType = autoware_vehicle_msgs::Shift::_data_type;
using TurnSignalType = autoware_vehicle_msgs::TurnSignal::_data_type;
using GateModeType = autoware_control_msgs::GateMode::_data_type;

class AutowareJoyControllerNode
{
public:
  AutowareJoyControllerNode();

private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  double update_rate_;
  double accel_ratio_;
  double brake_ratio_;
  double steer_ratio_;
  double steering_angle_velocity_;

  // ControlCommand Parameter
  double velocity_gain_;
  double max_forward_velocity_;
  double max_backward_velocity_;
  double backward_accel_ratio_;

  // Subscriber
  ros::Subscriber sub_joy_;
  ros::Subscriber sub_twist_;

  ros::Time last_joy_received_time_;
  std::shared_ptr<const JoyConverter> joy_;
  geometry_msgs::TwistStamped::ConstPtr twist_;

  void onJoy(const sensor_msgs::Joy::ConstPtr & msg);
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr & msg);

  // Publisher
  ros::Publisher pub_control_command_;
  ros::Publisher pub_raw_control_command_;
  ros::Publisher pub_shift_;
  ros::Publisher pub_turn_signal_;
  ros::Publisher pub_gate_mode_;
  ros::Publisher pub_emergency_;
  ros::Publisher pub_autoware_engage_;
  ros::Publisher pub_vehicle_engage_;

  void publishControlCommand();
  void publishRawControlCommand();
  void publishShift();
  void publishTurnSignal();
  void publishGateMode();
  void publishEmergency();
  void publishEngage();

  // Previous State
  autoware_control_msgs::ControlCommand prev_control_command_;
  autoware_vehicle_msgs::RawControlCommand prev_raw_control_command_;
  ShiftType prev_shift_ = autoware_vehicle_msgs::Shift::NONE;
  TurnSignalType prev_turn_signal_ = autoware_vehicle_msgs::TurnSignal::NONE;
  GateModeType prev_gate_mode_ = autoware_control_msgs::GateMode::AUTO;

  // Timer
  ros::Timer timer_;

  bool isDataReady();
  void onTimer(const ros::TimerEvent & event);
};

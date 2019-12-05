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

#ifndef LEXUS_CONVERTER_H
#define LEXUS_CONVERTER_H

#include <string>
#include <cstdlib>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>

#include <pacmod_msgs/SystemCmdFloat.h>
#include <pacmod_msgs/SteerSystemCmd.h>
#include <pacmod_msgs/SystemRptFloat.h>
#include <pacmod_msgs/WheelSpeedRpt.h>

#include <autoware_control_msgs/VehicleCommandStamped.h>
#include "accel_map.h"
#include "brake_map.h"

class LexusConverter
{
public:
  LexusConverter();
  ~LexusConverter();

  void run();

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // subscribers
  ros::Subscriber vehicle_cmd_sub_;
  ros::Subscriber engage_sub_;
  ros::Subscriber steer_wheel_sub_;
  ros::Subscriber wheel_speed_sub_;

  // publishers
  ros::Publisher accel_cmd_pub_;
  ros::Publisher brake_cmd_pub_;
  ros::Publisher steer_cmd_pub_;
  ros::Publisher vehicle_twist_pub_;

  // ros param
  std::string base_frame_id_;
  int command_timeout_;  // vehicle_cmd timeout [ms]
  double loop_rate_;     // [Hz]
  std::string topic_curr_vel_;
  std::string topic_vehicle_cmd_;
  double tire_radius_;  // [m]
  double wheel_base_;   // [m]
  double agr_coef_a_;
  double agr_coef_b_;
  double agr_coef_c_;
  std::string csv_path_accel_map_;  // file path to accel map csv file
  std::string csv_path_brake_map_;  // file path to brake map csv file
  double acc_emergency_;            // [m/s^2]
  double max_throttle_;
  double min_throttle_;
  double max_brake_;
  double min_brake_;
  double max_steering_wheel_;
  double min_steering_wheel_;

  // variables
  std::shared_ptr<autoware_control_msgs::VehicleCommandStamped> vehicle_cmd_ptr_;
  bool engage_;
  bool prev_engage_;
  bool vehicle_cmd_initialized_;
  bool wheel_speed_initialized_;
  bool steer_wheel_initialized_;
  ros::Time command_time_;
  autoware_control_msgs::VehicleCommandStamped vehicle_cmd_;
  ros::Rate* rate_;
  AccelMap accel_map_;
  BrakeMap brake_map_;
  double steer_wheel_;  // [rad]
  double wheel_speed_;  // [m/s]

  // callbacks
  void callbackVehicleCmd(const autoware_control_msgs::VehicleCommandStamped::ConstPtr& msg);
  void callbackEngage(const std_msgs::BoolConstPtr& msg);
  void callbackSteerWheel(const pacmod_msgs::SystemRptFloatConstPtr& msg);
  void callbackWheelSpeed(const pacmod_msgs::WheelSpeedRptConstPtr& msg);

  // functions
  void publishCommand();
};

#endif

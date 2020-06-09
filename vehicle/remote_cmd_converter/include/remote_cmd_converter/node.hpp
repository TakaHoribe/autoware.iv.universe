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

#ifndef REMOTE_CMD_CONVERTER_REMOTE_CMD_CONVERTER_NODE_HPP_
#define REMOTE_CMD_CONVERTER_REMOTE_CMD_CONVERTER_NODE_HPP_

#include <memory>
#include <string>

#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>

#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_control_msgs/GateMode.h>
#include <autoware_vehicle_msgs/RawControlCommand.h>
#include <autoware_vehicle_msgs/RawControlCommandStamped.h>
#include <autoware_vehicle_msgs/ShiftStamped.h>
#include <autoware_vehicle_msgs/VehicleCommand.h>

#include <raw_vehicle_cmd_converter/accel_map.h>
#include <raw_vehicle_cmd_converter/brake_map.h>

#include <diagnostic_updater/diagnostic_updater.h>

class RemoteCmdConverter
{
public:
  RemoteCmdConverter();
  ~RemoteCmdConverter() = default;

private:
  ros::NodeHandle nh_;              //!< @brief ros node handle
  ros::NodeHandle pnh_;             //!< @brief private ros node handle
  ros::Publisher pub_cmd_;          //!< @brief topic publisher for tlow level vehicle command
  ros::Publisher pub_current_cmd_;  //!< @brief for remote to check received time.
  ros::Subscriber sub_velocity_;    //!< @brief subscriber for currrent velocity
  ros::Subscriber sub_cmd_;         //!< @brief subscriber for vehicle command
  ros::Subscriber sub_emergency_;
  ros::Subscriber sub_shift_cmd_;
  ros::Subscriber sub_gate_mode_;

  std::shared_ptr<double> current_velocity_ptr_;  // [m/s]
  autoware_vehicle_msgs::ShiftStampedConstPtr current_shift_cmd_;
  bool current_emergency_cmd_;

  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool acc_map_initialized_;

  double ref_vel_gain_;  // reference velocity = current velocity + desired acceleration * gain

  void onRemoteCmd(const autoware_vehicle_msgs::RawControlCommandStampedConstPtr remote_cmd_ptr);
  void onVelocity(const geometry_msgs::TwistStampedConstPtr msg);
  void onShiftCmd(const autoware_vehicle_msgs::ShiftStampedConstPtr msg);
  void onEmergency(const std_msgs::Bool msg);
  void onGateMode(const autoware_control_msgs::GateModeConstPtr msg);
  double calculateAcc(const autoware_vehicle_msgs::RawControlCommand & cmd, const double vel);
  double getShiftVelocitySign(const autoware_vehicle_msgs::ShiftStamped & cmd);

  /* for Hz check */
  ros::Timer rate_check_timer_;
  diagnostic_updater::Updater updater_;  // for emergency handling
  autoware_control_msgs::GateModeConstPtr current_gate_mode_;
  std::shared_ptr<ros::Time> latest_cmd_received_time_;
  double time_threshold_;

  void onTimer(const ros::TimerEvent & event);
  void produceDiagnostics(diagnostic_updater::DiagnosticStatusWrapper & stat);
  bool checkRemoteTopicRate();
};

#endif  // REMOTE_CMD_CONVERTER_REMOTE_CMD_CONVERTER_NODE_HPP_

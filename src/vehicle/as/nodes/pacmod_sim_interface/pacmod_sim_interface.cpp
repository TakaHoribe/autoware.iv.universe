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

#include <string>
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
#include <autoware_control_msgs/VehicleStatusStamped.h>

const double STEERING_GEAR_RATIO = 18.0;
const double RAD2DEG = 180.0 / 3.14159265;

class PacmodSimInterface
{
public:
  PacmodSimInterface() : nh_(""), pnh_("~")
  {
    nh_.param<double>("/vehicle_info/wheel_radius", tire_radius_, double(0.341));
    timer_ = nh_.createTimer(ros::Duration(0.03), &PacmodSimInterface::timerCallback, this);
    sim_vehicle_cmd_pub_ = pnh_.advertise<geometry_msgs::TwistStamped>("/sim/vehicle_cmd", 1);
    steer_rpt_pub_ = pnh_.advertise<pacmod_msgs::SystemRptFloat>("/pacmod/parsed_tx/steer_rpt", 1);
    wheel_speed_rpt_pub_ = pnh_.advertise<pacmod_msgs::WheelSpeedRpt>("/pacmod/parsed_tx/wheel_speed_rpt", 1);
    sim_status_sub_ = pnh_.subscribe("/sim/status", 1, &PacmodSimInterface::callbackSimStatus, this);
    sim_velocity_sub_ = pnh_.subscribe("/sim/velocity", 1, &PacmodSimInterface::callbackSimVelocity, this);
    accel_cmd_sub_ = pnh_.subscribe("/pacmod/as_rx/accel_cmd", 1, &PacmodSimInterface::callbackAccelCmd, this);
    brake_cmd_sub_ = pnh_.subscribe("/pacmod/as_rx/brake_cmd", 1, &PacmodSimInterface::callbackBrakeCmd, this);
    steer_cmd_sub_ = pnh_.subscribe("/pacmod/as_rx/steer_cmd", 1, &PacmodSimInterface::callbackSteerCmd, this);

    current_accel_cmd_.enable = false;
    current_brake_cmd_.enable = false;
    current_steer_cmd_.enable = false;
  };

  ~PacmodSimInterface() = default;

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  // subscribers
  ros::Subscriber sim_status_sub_;
  ros::Subscriber sim_velocity_sub_;
  ros::Subscriber accel_cmd_sub_;
  ros::Subscriber brake_cmd_sub_;
  ros::Subscriber steer_cmd_sub_;

  // publishers
  ros::Publisher sim_vehicle_cmd_pub_;
  ros::Publisher steer_rpt_pub_;
  ros::Publisher wheel_speed_rpt_pub_;

  double tire_radius_;

  autoware_control_msgs::VehicleCommandStamped current_vehicle_cmd_;
  pacmod_msgs::SystemCmdFloat current_accel_cmd_;
  pacmod_msgs::SystemCmdFloat current_brake_cmd_;
  pacmod_msgs::SteerSystemCmd current_steer_cmd_;

  ros::Timer timer_;

  // callbacks
  void timerCallback(const ros::TimerEvent &te)
  {
    if (!current_accel_cmd_.enable || !current_brake_cmd_.enable || !current_steer_cmd_.enable)
    {
      return;
    }

    current_vehicle_cmd_.header.frame_id = "/base_link";
    current_vehicle_cmd_.header.stamp = ros::Time::now();
    current_vehicle_cmd_.command.gear.gear = autoware_control_msgs::Gear::DRIVE;
    current_vehicle_cmd_.command.turn_signal.signal = autoware_control_msgs::TurnSignal::NONE;
    current_vehicle_cmd_.command.control.header.frame_id = "/base_link";
    current_vehicle_cmd_.command.control.header.stamp = ros::Time::now();
    current_vehicle_cmd_.command.control.control.steering_angle = current_steer_cmd_.command / STEERING_GEAR_RATIO / RAD2DEG;
    current_vehicle_cmd_.command.control.control.steering_angle_velocity = 0.0;
    current_vehicle_cmd_.command.control.control.speed = 0.0;
    const double temp_coeff = 10.0;
    current_vehicle_cmd_.command.control.control.acceleration = (current_accel_cmd_.command - current_brake_cmd_.command) / 2.0 * temp_coeff;
    current_vehicle_cmd_.command.emergency = false;

    sim_vehicle_cmd_pub_.publish(current_vehicle_cmd_);
  };

  void callbackSimStatus(const autoware_control_msgs::VehicleStatusStamped &msg)
  {
    /* wheel ratation speed [rad/s] */
    pacmod_msgs::WheelSpeedRpt wheel_speed;
    wheel_speed.header = msg.header;
    double rotation_speed = msg.status.speed / tire_radius_;
    wheel_speed.front_left_wheel_speed = rotation_speed;
    wheel_speed.front_right_wheel_speed = rotation_speed;
    wheel_speed.rear_left_wheel_speed = rotation_speed;
    wheel_speed.rear_right_wheel_speed = rotation_speed;
    wheel_speed_rpt_pub_.publish(wheel_speed);

    /* steering wheel angle [deg] */
    pacmod_msgs::SystemRptFloat steer;
    steer.header = msg.header;
    steer.enabled = true;
    steer.override_active = false;
    steer.command_output_fault = false;
    steer.input_output_fault = false;
    steer.output_reported_fault = false;
    steer.pacmod_fault = false;
    steer.vehicle_fault = false;
    steer.manual_input = 0.0;
    steer.command = 0.0;
    steer.output = msg.status.steering_angle * STEERING_GEAR_RATIO * RAD2DEG;
    steer_rpt_pub_.publish(steer);
  };

  void callbackSimVelocity(const geometry_msgs::TwistStamped &msg){};
  void callbackAccelCmd(const pacmod_msgs::SystemCmdFloat &msg)
  {
    current_accel_cmd_ = msg;
  };

  void callbackBrakeCmd(const pacmod_msgs::SystemCmdFloat &msg)
  {
    current_brake_cmd_ = msg;
  };

  void callbackSteerCmd(const pacmod_msgs::SteerSystemCmd &msg)
  {
    current_steer_cmd_ = msg;
  };
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pacmod_sim_interface");
  PacmodSimInterface node;
  ros::spin();
  return 0;
};

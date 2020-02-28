/*
 *  Copyright (c) 2017, Tier IV, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "vehicle_cmd_gate/vehicle_cmd_gate.h"

VehicleCmdGate::VehicleCmdGate() : nh_(""), pnh_("~"), is_engaged_(false) {
  vehicle_cmd_pub_ = pnh_.advertise<autoware_vehicle_msgs::VehicleCommandStamped>("output/vehicle_cmd", 1, true);
  shift_cmd_pub_ = pnh_.advertise<autoware_vehicle_msgs::Shift>("output/shift_cmd", 1, true);
  control_cmd_sub_ = pnh_.subscribe("input/control_cmd", 1, &VehicleCmdGate::ctrlCmdCallback, this);
  engage_sub_ = pnh_.subscribe("input/engage", 1, &VehicleCmdGate::engageCallback, this);

  double wheel_base, vel_lim, lon_acc_lim, lon_jerk_lim, lat_acc_lim, lat_jerk_lim;
  pnh_.param<double>("/vehicle_info/wheel_base", wheel_base, 2.79);
  pnh_.param<double>("vel_lim", vel_lim, 25.0);
  pnh_.param<double>("lon_acc_lim", lon_acc_lim, 5.0);
  pnh_.param<double>("lon_jerk_lim", lon_jerk_lim, 5.0);
  pnh_.param<double>("lat_acc_lim", lat_acc_lim, 5.0);
  pnh_.param<double>("lat_jerk_lim", lat_jerk_lim, 5.0);
  filter.setWheelBase(wheel_base);
  filter.setVelLim(vel_lim);
  filter.setLonAccLim(lon_acc_lim);
  filter.setLonJerkLim(lon_jerk_lim);
  filter.setLatAccLim(lat_acc_lim);
  filter.setLatJerkLim(lat_jerk_lim);
}

void VehicleCmdGate::engageCallback(const std_msgs::Bool msg) { is_engaged_ = msg.data; }

double VehicleCmdGate::getDt() {
  double dt = 0.0;
  if (!prev_time_) {
    prev_time_ = std::make_shared<ros::Time>(ros::Time::now());
  } else {
    ros::Time curr_time = ros::Time::now();
    dt = (curr_time - *prev_time_).toSec();
    *prev_time_ = curr_time;
  }
  return dt;
}

void VehicleCmdGate::ctrlCmdCallback(const autoware_control_msgs::ControlCommandStamped &input_msg) {
  autoware_vehicle_msgs::VehicleCommandStamped cmd;
  cmd.header = input_msg.header;

  if (!is_engaged_) {
    cmd.command.control.steering_angle = prev_vehicle_cmd_.command.control.steering_angle;
    cmd.command.control.steering_angle_velocity = prev_vehicle_cmd_.command.control.steering_angle_velocity;
    cmd.command.control.velocity = 0.0;
    cmd.command.control.acceleration = -1.5;
  } else {
    cmd.command.control = input_msg.control;
  }

  double dt = getDt();

  /* limit filtering */
  filter.limitLongitudinalWithVel(cmd.command.control);
  filter.limitLongitudinalWithAcc(dt, cmd.command.control);
  filter.limitLongitudinalWithJerk(dt, cmd.command.control);
  filter.limitLateralWithLatAcc(dt, cmd.command.control);
  filter.limitLateralWithLatJerk(dt, cmd.command.control);

  /* publish vehicle cmd */
  vehicle_cmd_pub_.publish(cmd);

  prev_vehicle_cmd_ = cmd;
  filter.setPrevCmd(cmd.command.control);

  /* publish shift command */
  autoware_vehicle_msgs::Shift shift_msg;
  shift_msg.header.frame_id = "base_link";
  shift_msg.header.stamp = ros::Time::now();
  shift_msg.data = cmd.command.control.velocity >= 0.0 ? autoware_vehicle_msgs::Shift::DRIVE
                                                                       : autoware_vehicle_msgs::Shift::REVERSE;
  shift_cmd_pub_.publish(shift_msg);
}

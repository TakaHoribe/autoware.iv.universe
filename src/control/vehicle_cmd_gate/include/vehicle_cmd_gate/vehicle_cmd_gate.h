/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#ifndef VEHICLE_CMD_GATE_VEHICLE_CMD_GATE_H
#define VEHICLE_CMD_GATE_VEHICLE_CMD_GATE_H


#include <ros/ros.h>

#include "autoware_control_msgs/ControlCommandStamped.h"
#include "autoware_control_msgs/VehicleCommandStamped.h"


class VehicleCmdGate
{

public:
  VehicleCmdGate();
  ~VehicleCmdGate() = default;

private:

  void ctrlCmdCallback(const autoware_control_msgs::ControlCommandStamped::ConstPtr& msg);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Publisher vehicle_cmd_pub_;
  ros::Subscriber control_cmd_sub_;


};

#endif  // VEHICLE_CMD_GATE_VEHICLE_CMD_GATE_H

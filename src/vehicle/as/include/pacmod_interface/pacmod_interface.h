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

#ifndef VEHICLE_AS_PACMOD_INTERFACE_H
#define VEHICLE_AS_PACMOD_INTERFACE_H

#include <string>
#include <cstdlib>
#include <algorithm>
#include <cmath>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/TwistStamped.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <pacmod_msgs/SystemCmdFloat.h>
#include <pacmod_msgs/SteerSystemCmd.h>
#include <pacmod_msgs/SystemRptFloat.h>
#include <pacmod_msgs/SystemRptInt.h>
#include <pacmod_msgs/GlobalRpt.h>
#include <pacmod_msgs/WheelSpeedRpt.h>

#include <autoware_control_msgs/VehicleCommandStamped.h>

#include <accel_map_converter/accel_map.h>
#include <accel_map_converter/brake_map.h>

class PacmodInterface
{
public:
  PacmodInterface();
  ~PacmodInterface();

  void run();

private:
  typedef message_filters::sync_policies::ApproximateTime<
      pacmod_msgs::SystemRptFloat, pacmod_msgs::WheelSpeedRpt, pacmod_msgs::SystemRptFloat,
      pacmod_msgs::SystemRptFloat, pacmod_msgs::SystemRptInt, pacmod_msgs::GlobalRpt>
      PacmodFeedbacksSyncPolicy;

  /* handle */
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  /* subscribers */
  ros::Subscriber vehicle_cmd_sub_;
  ros::Subscriber shift_cmd_sub_;
  ros::Subscriber engage_cmd_sub_;
  message_filters::Subscriber<pacmod_msgs::SystemRptFloat> *steer_wheel_sub_;
  message_filters::Subscriber<pacmod_msgs::WheelSpeedRpt> *wheel_speed_sub_;
  message_filters::Subscriber<pacmod_msgs::SystemRptFloat> *accel_rpt_sub_;
  message_filters::Subscriber<pacmod_msgs::SystemRptFloat> *brake_rpt_sub_;
  message_filters::Subscriber<pacmod_msgs::SystemRptInt> *shift_rpt_sub_;
  message_filters::Subscriber<pacmod_msgs::GlobalRpt> *global_rpt_sub_;
  message_filters::Synchronizer<PacmodFeedbacksSyncPolicy> *pacmod_feedbacks_sync_;

  /* publishers */
  ros::Publisher accel_cmd_pub_;
  ros::Publisher brake_cmd_pub_;
  ros::Publisher steer_cmd_pub_;
  ros::Publisher vehicle_twist_pub_;

  /* ros param */
  ros::Rate *rate_;
  std::string base_frame_id_;
  int command_timeout_ms_; // vehicle_cmd timeout [ms]
  bool is_pacmod_rpt_received_;
  bool is_pacmod_enabled_;
  bool is_clear_override_needed_;
  bool show_debug_info_;
  double loop_rate_;   // [Hz]
  double tire_radius_; // [m]
  double wheel_base_;  // [m]
  double vgr_coef_a_;  // variable gear ratio coeffs
  double vgr_coef_b_;  // variable gear ratio coeffs
  double vgr_coef_c_;  // variable gear ratio coeffs

  double acc_emergency_;           // acceleration when emergency [m/s^2]
  double max_throttle_;            // max throttle [0~1]
  double max_brake_;               // max throttle [0~1]
  double max_steering_wheel_;      // max steering wheel angle [rad]

  /* input values */
  std::shared_ptr<autoware_control_msgs::VehicleCommandStamped> vehicle_cmd_ptr_;
  std::shared_ptr<autoware_control_msgs::Shift> shift_cmd_ptr_;

  std::shared_ptr<pacmod_msgs::SystemRptFloat> steer_wheel_rpt_ptr_; // [rad]
  std::shared_ptr<pacmod_msgs::WheelSpeedRpt> wheel_speed_rpt_ptr_;  // [m/s]
  std::shared_ptr<pacmod_msgs::SystemRptFloat> accel_rpt_ptr_;
  std::shared_ptr<pacmod_msgs::SystemRptFloat> brake_rpt_ptr_; // [m/s]
  std::shared_ptr<pacmod_msgs::SystemRptInt> shift_rpt_ptr_;   // [m/s]
  std::shared_ptr<pacmod_msgs::GlobalRpt> global_rpt_ptr_;     // [m/s]
  bool engage_cmd_;
  bool prev_engage_cmd_;
  ros::Time command_received_time_;

  AccelMap accel_map_;
  BrakeMap brake_map_;
  bool acc_map_initialized_;

  /* callbacks */
  void callbackVehicleCmd(const autoware_control_msgs::VehicleCommandStamped::ConstPtr &msg);
  void callbackShiftCmd(const autoware_control_msgs::Shift::ConstPtr &msg);
  void callbackEngage(const std_msgs::BoolConstPtr &msg);
  void callbackPacmodRpt(const pacmod_msgs::SystemRptFloatConstPtr &steer_wheel_rpt,
                         const pacmod_msgs::WheelSpeedRptConstPtr &wheel_speed_rpt,
                         const pacmod_msgs::SystemRptFloatConstPtr &accel_rpt,
                         const pacmod_msgs::SystemRptFloatConstPtr &brake_rpt,
                         const pacmod_msgs::SystemRptIntConstPtr &shift_rpt,
                         const pacmod_msgs::GlobalRptConstPtr &global_rpt);

  /*  functions */
  void publishCommands();
  void publishShiftCmd();
  double calculateVehicleVelocity(const pacmod_msgs::WheelSpeedRpt &wheel_speed_rpt, const pacmod_msgs::SystemRptInt& shift_rpt);
  bool calculateAccelMap(const double curr_wheel_speed, const double &desired_acc, double &desired_throttle, double &desired_brake);
  double calculateVariableGearRatio(const double vel, const double steer_wheel);
};

#endif

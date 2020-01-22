/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

/**
 * @file moc_follower.h
 * @brief mpc follower class
 * @author Takamasa Horibe
 * @date 2019.05.01
 */

#pragma once
#include <vector>
#include <iostream>
#include <limits>
#include <chrono>
#include <unistd.h>
#include <deque>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_control_msgs/ControlCommandStamped.h>
#include <autoware_vehicle_msgs/Steering.h>

#include "mpc_follower/mpc_utils.h"
#include "mpc_follower/mpc_trajectory.h"
#include "mpc_follower/lowpass_filter.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_dynamics.h"
#include "mpc_follower/vehicle_model/vehicle_model_bicycle_kinematics_no_delay.h"
#include "mpc_follower/qp_solver/qp_solver_unconstr_fast.h"
// #include "mpc_follower/qp_solver/qp_solver_qpoases.h"

/** 
 * @class MPC-based waypoints follower class
 * @brief calculate control command to follow reference waypoints
 */
class MPCFollower
{
public:
  /**
   * @brief constructor
   */
  MPCFollower();

  /**
   * @brief destructor
   */
  ~MPCFollower();

private:
  ros::NodeHandle nh_;                    //!< @brief ros node handle
  ros::NodeHandle pnh_;                   //!< @brief private ros node handle
  ros::Publisher pub_steer_vel_ctrl_cmd_; //!< @brief topic publisher for control command
  ros::Publisher pub_twist_cmd_;          //!< @brief topic publisher for twist command
  ros::Subscriber sub_ref_path_;          //!< @brief topic subscriber for reference waypoints
  ros::Subscriber sub_steering_;          //!< @brief subscriber for currrent steering
  ros::Subscriber sub_current_vel_;
  ros::Timer timer_control_; //!< @brief timer for control command computation

  MPCTrajectory ref_traj_;                                   //!< @brief reference trajectory to be followed
  Butterworth2dFilter lpf_steering_cmd_;                     //!< @brief lowpass filter for steering command
  Butterworth2dFilter lpf_lateral_error_;                    //!< @brief lowpass filter for lateral error to calculate derivatie
  Butterworth2dFilter lpf_yaw_error_;                        //!< @brief lowpass filter for heading error to calculate derivatie
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_; //!< @brief vehicle model for MPC
  std::string vehicle_model_type_;                           //!< @brief vehicle model type for MPC
  std::shared_ptr<QPSolverInterface> qpsolver_ptr_;          //!< @brief qp solver for MPC
  std::string output_interface_;                             //!< @brief output command type
  std::deque<double> input_buffer_;                          //!< @brief control input (mpc_output) buffer for delay time conpemsation

  /* parameters for control*/
  double ctrl_period_;              //!< @brief control frequency [s]
  double steering_lpf_cutoff_hz_;   //!< @brief cutoff frequency of lowpass filter for steering command [Hz]
  double admisible_position_error_; //!< @brief stop MPC calculation when lateral error is large than this value [m]
  double admisible_yaw_error_deg_;  //!< @brief stop MPC calculation when heading error is large than this value [deg]
  double steer_lim_deg_;            //!< @brief steering command limit [rad]
  double wheelbase_;                //!< @brief vehicle wheelbase length [m] to convert steering angle to angular velocity

  /* parameters for path smoothing */
  bool enable_path_smoothing_;     //< @brief flag for path smoothing
  bool enable_yaw_recalculation_;  //< @brief flag for recalculation of yaw angle after resampling
  int path_filter_moving_ave_num_; //< @brief param of moving average filter for path smoothing
  int path_smoothing_times_;       //< @brief number of times of applying path smoothing filter
  int curvature_smoothing_num_;    //< @brief point-to-point index distance used in curvature calculation
  double traj_resample_dist_;      //< @brief path resampling interval [m]

  struct MPCParam
  {
    int prediction_horizon;                         //< @brief prediction horizon step
    double prediction_sampling_time;                //< @brief prediction horizon period
    double weight_lat_error;                        //< @brief lateral error weight in matrix Q
    double weight_heading_error;                    //< @brief heading error weight in matrix Q
    double weight_heading_error_squared_vel_coeff;  //< @brief heading error * velocity weight in matrix Q
    double weight_steering_input;                   //< @brief steering error weight in matrix R
    double weight_steering_input_squared_vel_coeff; //< @brief steering error * velocity weight in matrix R
    double weight_lat_jerk;                         //< @brief lateral jerk weight in matrix R
    double weight_terminal_lat_error;               //< @brief terminal lateral error weight in matrix Q
    double weight_terminal_heading_error;           //< @brief terminal heading error weight in matrix Q
    double zero_ff_steer_deg;                       //< @brief threshold that feed-forward angle becomes zero
    double delay_compensation_time;                 //< @brief delay time for steering input to be compensated
  };
  MPCParam mpc_param_; // for mpc design parameter

  std::shared_ptr<geometry_msgs::PoseStamped> current_pose_ptr_;      //!< @brief current measured pose
  std::shared_ptr<geometry_msgs::TwistStamped> current_velocity_ptr_; //!< @brief current measured pose
  std::shared_ptr<double> current_steer_ptr_;                         //!< @brief current measured pose
  autoware_planning_msgs::Trajectory current_trajectory_;             //!< @brief current waypoints to be followed

  double steer_cmd_prev_;     //< @brief steering command calculated in previous period
  double lateral_error_prev_; //< @brief previous lateral error for derivative
  double yaw_error_prev_;     //< @brief previous lateral error for derivative

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_; //!< @brief tf listener

  /**
   * @brief compute and publish control command for path follow with a constant control period
   */
  void timerCallback(const ros::TimerEvent &);

  /**
   * @brief set current_trajectory_ with receved message
   */
  void callbackRefPath(const autoware_planning_msgs::Trajectory::ConstPtr &);

  /**
   * @brief update current_pose from tf
   */
  void updateCurrentPose();

  /**
   * @brief set curent_steer with receved message
   */
  void callbackSteering(const autoware_vehicle_msgs::Steering &msg);

  /**
   * @brief set current_velocity with receved message
   */
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg);

  /**
   * @brief publish control command as autoware_msgs/ControlCommand type
   * @param [in] vel_cmd velocity command [m/s] for vehicle control
   * @param [in] acc_cmd acceleration command [m/s2] for vehicle control
   * @param [in] steer_cmd steering angle command [rad] for vehicle control
   */
  void publishCtrlCmd(const double &vel_cmd, const double &acc_cmd, const double &steer_cmd, const double &steer_vel_cmd);

  /**
   * @brief calculate control command by MPC algorithm
   * @param [out] vel_cmd velocity command
   * @param [out] acc_cmd acceleration command
   * @param [out] steer_cmd steering command
   * @param [out] steer_vel_cmd steering rotation speed command
   */
  bool calculateMPC(double &vel_cmd, double &acc_cmd, double &steer_cmd, double &steer_vel_cmd);

  /* debug */
  bool show_debug_info_; //!< @brief flag to display debug info

  ros::Publisher pub_debug_marker_;
  ros::Publisher pub_debug_values_;        //!< @brief publisher for debug info
  ros::Publisher pub_debug_mpc_calc_time_; //!< @brief publisher for debug info

  ros::Subscriber sub_estimate_twist_;         //!< @brief subscriber for /estimate_twist for debug
  geometry_msgs::TwistStamped estimate_twist_; //!< @brief received /estimate_twist for debug

  /**
   * @brief convert MPCTraj to visualizaton marker for visualization
   */
  void convertTrajToMarker(const MPCTrajectory &traj, visualization_msgs::MarkerArray &markers,
                           std::string ns, double r, double g, double b, double z);

  /**
   * @brief callback for estimate twist for debug
   */
  void callbackEstimateTwist(const geometry_msgs::TwistStamped &msg) { estimate_twist_ = msg; }
};

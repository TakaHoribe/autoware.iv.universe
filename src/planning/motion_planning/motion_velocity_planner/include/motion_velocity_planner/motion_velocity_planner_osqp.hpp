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
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/utils.h>
#include <boost/shared_ptr.hpp>
#include <tf2_ros/transform_listener.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <geometry_msgs/TwistStamped.h>

#include <dynamic_reconfigure/server.h>
#include <motion_velocity_planner/MotionVelocityPlannerConfig.h>

#include <motion_velocity_planner/motion_velocity_planner_utils.hpp>

#include <stop_planner/planning_utils.h>
#include <osqp_interface/osqp_interface.h>


class MotionVelocityPlanner
{
public:
  MotionVelocityPlanner();
  ~MotionVelocityPlanner();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_dist_to_stopline_;
  ros::Subscriber sub_current_velocity_;
  ros::Subscriber sub_current_trajectory_;
  ros::Subscriber sub_external_velocity_limit_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;       //!< @brief tf listener

  boost::shared_ptr<geometry_msgs::PoseStamped const> current_pose_ptr_;          // current vehicle pose
  boost::shared_ptr<geometry_msgs::TwistStamped const> current_velocity_ptr_;     // current vehicle twist
  boost::shared_ptr<autoware_planning_msgs::Trajectory const> base_traj_raw_ptr_; // current base_waypoints
  boost::shared_ptr<std_msgs::Float32 const> external_velocity_limit_ptr_;        // current external_velocity_limit

  autoware_planning_msgs::Trajectory prev_output_trajectory_;  // velocity replanned waypoints (output of this node)



  osqp::OSQPInterface qp_solver_;

  bool show_debug_info_;      // printF level 1
  bool show_debug_info_all_;  // print level 2
  bool show_figure_;          // for plot visualize
  bool publish_debug_trajs_;

  struct MotionVelocityPlannerParam
  {
    double max_velocity;          // max velocity [m/s]
    double max_accel;             // max acceleration in planning [m/s2] > 0
    double min_decel;             // min deceltion in planning [m/s2] < 0
    double max_lateral_accel;           // max lateral acceleartion [m/ss] > 0
    double replan_vel_deviation;  // replan with current speed if speed deviation exceeds this value [m/s]
    double engage_velocity;       // use this speed when start moving [m/s]
    double engage_acceleration;   // use this acceleration when start moving [m/ss]
    double extract_ahead_dist;    // forward waypoints distance from current position [m]
    double extract_behind_dist;   // backward waypoints distance from current position [m]
    double max_trajectory_length; // max length of the objective trajectory for resample
    double min_trajectory_length; // min length of the objective trajectory for resample
    double resample_total_time;   // max time to calculate trajectory length
    double resample_dt;           // dt to calculate trajectory length
    double min_trajectory_interval_distance; // minimum interval distance between each trajectory points
    double stop_dist_not_to_drive_vehicle; // set zero vel when vehicle stops and stop dist is closer than this
    double stop_dist_mergin;
  } planning_param_;

  struct QPParam
  {
    double pseudo_jerk_weight;
    double over_v_weight;
    double over_a_weight;
  } qp_param_;


  /* topic callback */
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr msg);
  void callbackCurrentTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr msg);
  void callbackExternalVelocityLimit(const std_msgs::Float32::ConstPtr msg);

  /* non-const methods */
  void run();
  void updateCurrentPose();

  void replanVelocity(const autoware_planning_msgs::Trajectory &input, const int input_closest,
                      const autoware_planning_msgs::Trajectory &prev_output_traj, const int prev_output_closest,
                      const std::vector<double> &interval_dist_arr, autoware_planning_msgs::Trajectory &output);
  void calcInitialMotion(const double &base_speed, const autoware_planning_msgs::Trajectory &base_waypoints, const int base_closest,
                         const autoware_planning_msgs::Trajectory &prev_replanned_traj, const int prev_replanned_traj_closest,
                         double &initial_vel, double &initial_acc, int &init_type) const;

  bool resampleTrajectory(const autoware_planning_msgs::Trajectory &input, autoware_planning_msgs::Trajectory &output,
                          std::vector<double> &interval_dist_arr) const;

  bool lateralAccelerationFilter(const autoware_planning_msgs::Trajectory &input,
                                 const double &max_lateral_accel, const unsigned int curvature_calc_idx_dist,
                                 autoware_planning_msgs::Trajectory &output) const;
  void publishTrajectory(const autoware_planning_msgs::Trajectory &traj) const;
  void preventMoveToVeryCloseStopLine(const int closest, const double move_dist_min, autoware_planning_msgs::Trajectory &trajectory) const;
  void publishStopDistance(const autoware_planning_msgs::Trajectory &trajectory, const int closest) const;

  void optimizeVelocity(const double initial_vel, const double initial_acc, const autoware_planning_msgs::Trajectory &input, const int closest,
                        const std::vector<double> &interval_dist_arr, autoware_planning_msgs::Trajectory &output);

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<motion_velocity_planner::MotionVelocityPlannerConfig> dyncon_server_;
  void dynamicRecofCallback(motion_velocity_planner::MotionVelocityPlannerConfig &config, uint32_t level)
  {
    planning_param_.max_accel = config.max_accel;
    planning_param_.min_decel = config.min_decel;
    planning_param_.engage_velocity = config.engage_velocity;
    planning_param_.engage_acceleration = config.engage_acceleration;
  }

  /* debug */
  ros::Publisher pub_trajectory_raw_;
  ros::Publisher pub_trajectory_vel_lim_;
  ros::Publisher pub_trajectory_latcc_filtered_;
  ros::Publisher pub_trajectory_resampled_;
  ros::Publisher debug_closest_velocity_;
  void publishClosestVelocity(const double &vel) const;
};

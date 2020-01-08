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
#include <velocity_planner/VelocityPlannerConfig.h>

#include <velocity_planner/velocity_planner_utils.hpp>

#include <stop_planner/planning_utils.h>


// #define USE_MATPLOTLIB_FOR_VELOCITY_VIZ
#ifdef USE_MATPLOTLIB_FOR_VELOCITY_VIZ
#include "matplotlibcpp.h"
#endif

class VelocityPlanner
{
public:
  VelocityPlanner();
  ~VelocityPlanner();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher pub_trajectory_;
  ros::Publisher pub_dist_to_stopline_;
  ros::Subscriber sub_current_velocity_;
  ros::Subscriber sub_current_trajectory_;
  ros::Subscriber sub_external_velocity_limit_;

  boost::shared_ptr<geometry_msgs::PoseStamped const> current_pose_ptr_;          // current vehicle pose
  boost::shared_ptr<geometry_msgs::TwistStamped const> current_velocity_ptr_;     // current vehicle twist
  boost::shared_ptr<autoware_planning_msgs::Trajectory const> base_traj_raw_ptr_; // current base_waypoints
  boost::shared_ptr<std_msgs::Float32 const> external_velocity_limit_ptr_;        // current external_velocity_limit

  autoware_planning_msgs::Trajectory prev_output_trajectory_;  // velocity replanned waypoints (output of this node)

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;       //!< @brief tf listener

  bool show_debug_info_;      // printF level 1
  bool show_debug_info_all_;  // print level 2
  bool show_figure_;          // for plot visualize
  bool enable_latacc_filter_;
  bool enable_to_publish_emergency_;

  struct Motion
  {
    double vel;
    double acc;
    double jerk;
    Motion(double v = 0.0, double a = 0.0, double s = 0.0) : vel(v), acc(a), jerk(s){};
    Motion &operator=(const Motion &m)
    {
      vel = m.vel;
      acc = m.acc;
      jerk = m.jerk;
      return *this;
    };
  };

  struct VelocityPlannerParam
  {
    double max_velocity;                   // max velocity [m/s]
    double max_accel;                      // max acceleration in planning [m/s2] > 0
    double min_decel;                      // min deceltion in planning [m/s2] < 0
    double max_lat_acc;                    // max lateral acceleartion [m/ss] > 0
    double replan_vel_deviation;           // replan with current speed if speed deviation exceeds this value [m/s]
    double engage_velocity;                // use this speed when start moving [m/s]
    double engage_acceleration;            // use this acceleration when start moving [m/ss]
    double extract_ahead_dist;             // forward waypoints distance from current position [m]
    double extract_behind_dist;            // backward waypoints distance from current position [m]
    double stop_dist_not_to_drive_vehicle; // set zero vel when vehicle stops and stop dist is closer than this
    double emergency_flag_vel_thr_kmph;    // Threshold for throwing emergency flag when unable to stop under jerk constraints
    double stop_dist_mergin;
  } planning_param_;

  struct QPParam
  {
    double pseudo_jerk_weight;
  } qp_param_;


  /* topic callback */
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr msg);
  void callbackCurrentTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr msg);
  void callbackExternalVelocityLimit(const std_msgs::Float32::ConstPtr msg);

  /* non-const methods */
  void run();
  void updateCurrentPose();

  /* const methods */

  void replanVelocity(const autoware_planning_msgs::Trajectory &input, const int input_closest,
                      const autoware_planning_msgs::Trajectory &prev_output_traj, const int prev_output_closest, const double ds, 
                      autoware_planning_msgs::Trajectory &output) const;
  void calcInitialMotion(const double &base_speed, const autoware_planning_msgs::Trajectory &base_waypoints, const int base_closest,
                         const autoware_planning_msgs::Trajectory &prev_replanned_traj, const int prev_replanned_traj_closest,
                         VelocityPlanner::Motion *initial_motion, int &init_type) const;

  void plotWaypoint(const autoware_planning_msgs::Trajectory &trajectory, const std::string &color_str, const std::string &label_str) const;
  bool resampleTrajectory(const autoware_planning_msgs::Trajectory &input, autoware_planning_msgs::Trajectory &output, double &ds) const;

  bool lateralAccelerationFilter(const autoware_planning_msgs::Trajectory &input,
                                 const double &max_lat_acc, const unsigned int curvature_calc_idx_dist,
                                 autoware_planning_msgs::Trajectory &output) const;
  void publishTrajectory(const autoware_planning_msgs::Trajectory &traj) const;
  void preventMoveToVeryCloseStopLine(const int closest, const double move_dist_min, autoware_planning_msgs::Trajectory &trajectory) const;
  void publishStopDistance(const autoware_planning_msgs::Trajectory &trajectory, const int closest) const;

  void optimizeVelocity(const Motion initial_motion, const autoware_planning_msgs::Trajectory &input, const int closest,
                        const double ds, autoware_planning_msgs::Trajectory &output) const;

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<velocity_planner::VelocityPlannerConfig> dyncon_server_;
  
  void dynamicRecofCallback(velocity_planner::VelocityPlannerConfig &config, uint32_t level)
  {
    planning_param_.max_accel = config.max_accel;
    planning_param_.min_decel = config.min_decel;
    planning_param_.engage_velocity = config.engage_velocity;
    planning_param_.engage_acceleration = config.engage_acceleration;
  }


  /* debug */
  ros::Publisher debug_closest_velocity_;
  void publishClosestVelocity(const double &vel) const;

#ifdef USE_MATPLOTLIB_FOR_VELOCITY_VIZ
  void plotVelocity(const std::string &color_str, const autoware_planning_msgs::Trajectory &traj) const;
  void plotAcceleration(const std::string &color_str, const autoware_planning_msgs::Trajectory &traj) const;
  void plotJerk(const std::string &color_str, const autoware_planning_msgs::Trajectory &traj) const;
  void plotAll(const int &stop_idx_zero_vel, const int &input_closest, const autoware_planning_msgs::Trajectory &base,
               const autoware_planning_msgs::Trajectory &jerk_filtered) const;
#endif
};

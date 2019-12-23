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
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <boost/shared_ptr.hpp>
#include <iostream>

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
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_trajectory_, pub_is_emergency_, pub_dist_to_stopline_;
  ros::Subscriber sub_current_velocity_, sub_stop_fact_, sub_current_trajectory_, sub_external_velocity_limit_;
  ros::Timer timer_replan_;

  boost::shared_ptr<geometry_msgs::PoseStamped const> current_pose_ptr_;       // current vehicle pose
  boost::shared_ptr<geometry_msgs::TwistStamped const> current_velocity_ptr_;  // current vehicle twist
  boost::shared_ptr<autoware_planning_msgs::Trajectory const> base_traj_raw_ptr_;              // current base_waypoints
  boost::shared_ptr<std_msgs::Float32 const> external_velocity_limit_ptr_;     // current external_velocity_limit

  autoware_planning_msgs::Trajectory replanned_traj_;  // velocity replanned waypoints (output of this node)

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;       //!< @brief tf listener

  bool show_debug_info_;      // print level 1
  bool show_debug_info_all_;  // print level 2
  bool show_figure_;          // for plot visualize
  bool enable_latacc_filter_;
  double prev_stop_planning_jerk_;
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
  std::vector<Motion> replanned_traj_motion_;  // planning info of velocity, acceleration, jerk.

  struct VelocityPlannerParam
  {
    double max_velocity;
    double max_accel;             // max acceleration in planning [m/s2] > 0
    double min_decel;             // min deceltion in planning [m/s2] < 0
    double acc_jerk;          // max jerk in acceleration [m/s3] > 0
    double dec_jerk_nominal;  // nominal jerk in deceleration [m/s3] < 0
    double dec_jerk_urgent;   // min jerk in deceleration [m/s3] < 0
    double max_lat_acc;
    double replan_vel_deviation;           // replan with current speed if speed deviation exceeds this value [m/s]
    double replan_stop_point_change_dist;  // replan if stop position changes over this distance [m]
    double engage_velocity;                // use this speed when start moving [m/s]
    double engage_acceleration;            // use this acceleration when start moving [m/ss]
    double stopping_speed;                 // Speed just before stop point (speed at stop point is 0) [m/s]
    double extract_ahead_dist;             // forward waypoints distance from current position [m]
    double extract_behind_dist;            // backward waypoints distance from current position [m]
    int resample_num;          // used in resample with interpolate (output waypints size = original size * this value)
    double large_jerk_report;  // publish emergency topic if stop jerk overs this value
    double velocity_feedback_gain;  // to calculate desired acceleration from velocity deviation
    double stop_dist_not_to_drive_vehicle; // set zero vel when vehicle stops and stop dist is closer than this
    double emergency_flag_vel_thr_kmph;    // Threshold for throwing emergency flag when unable to stop under jerk constraints
    double jerk_planning_span;             // Interval jerk value when planning from nominal jerk to maximum jerk for stop
    double stop_dist_mergin;
  } planning_param_;

  /* topic callback */
  void callbackCurrentPose(const geometry_msgs::PoseStamped::ConstPtr msg);
  void callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr msg);
  void callbackCurrentTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr msg);
  void callbackExternalVelocityLimit(const std_msgs::Float32::ConstPtr msg);

  /* non-const methods */
  void timerReplanCallback(const ros::TimerEvent &e);
  void updateCurrentPose();

  /* const methods */
  void replanVelocity(const autoware_planning_msgs::Trajectory &input, const int input_closest,
                      const std::vector<Motion> &prev_output_motion, const int prev_output_closest,
                      autoware_planning_msgs::Trajectory &output, std::vector<Motion> &output_motion,
                      double &stop_planning_jerk) const;
  void calcInitialMotion(const double &base_speed, const autoware_planning_msgs::Trajectory &base_waypoints, const int base_closest,
                         const std::vector<Motion> &prev_replanned_traj_motion, const int prev_replanned_traj_closest,
                         VelocityPlanner::Motion *initial_motion, int &init_type) const;
  Motion updateMotionWithConstraint(const Motion &m_prev, const Motion &m_des_prev, const double &dt,
                                    const VelocityPlannerParam &planning_param, int &debug) const;

  void plotWaypoint(const autoware_planning_msgs::Trajectory &trajectory, const std::string &color_str, const std::string &label_str) const;
  bool moveAverageFilter(const autoware_planning_msgs::Trajectory &latacc_filtered_traj, const unsigned int move_ave_num,
                         autoware_planning_msgs::Trajectory &moveave_filtered_traj) const;
  bool lateralAccelerationFilter(const autoware_planning_msgs::Trajectory &input, const double &max_lat_acc,
                                 const unsigned int idx_dist, autoware_planning_msgs::Trajectory &latacc_filtered_traj) const;
  void jerkVelocityFilter(const Motion &initial_motion, const autoware_planning_msgs::Trajectory &input, const int input_closest,
                          const std::vector<Motion> &prev_output_motion, const int prev_output_closest,
                          const int &init_type, autoware_planning_msgs::Trajectory &output, std::vector<Motion> &output_motion) const;
  bool stopVelocityFilterWithMergin(const double &stop_mergin, const int &input_stop_idx,
                                    const autoware_planning_msgs::Trajectory &input, const std::vector<Motion> &input_motion,
                                    const int &input_closest, const double &planning_jerk, autoware_planning_msgs::Trajectory &output,
                                    bool &is_stop_ok, std::vector<Motion> &output_motion) const;
  bool stopVelocityFilterWithJerkRange(const double &jerk_max, const double &jerk_min, const double &jerk_span,
                                       const double &stop_mergin, const int &input_stop_idx,
                                       const autoware_planning_msgs::Trajectory &input, const std::vector<Motion> &input_motion,
                                       const int &input_closest, double &planning_jerk, autoware_planning_msgs::Trajectory &output,
                                       bool &is_stop_ok, std::vector<Motion> &output_motion) const;
  bool stopVelocityFilter(const int &input_stop_idx, const autoware_planning_msgs::Trajectory &input,
                          const std::vector<Motion> &input_motion, const int &input_closest,
                          const double &planning_jerk, autoware_planning_msgs::Trajectory &output, bool &is_stop_ok,
                          std::vector<Motion> &output_motion) const;
  void calculateMotionsFromWaypoints(const autoware_planning_msgs::Trajectory &trajectory, std::vector<Motion> motions) const;
  void publishTrajectory(const autoware_planning_msgs::Trajectory &traj) const;
  void insertZeroMotionsAfterIdx(const int &idx, std::vector<Motion> &motions) const;
  void setZeroLaneAndMotions(const autoware_planning_msgs::Trajectory &base_trajectory, autoware_planning_msgs::Trajectory &trajectory,
                             std::vector<Motion> &motions) const;
  void publishIsEmergency(const double &jerk_value) const;
  void preventMoveToVeryCloseStopLine(const int closest, autoware_planning_msgs::Trajectory &trajectory) const;
  void publishStopDistance(const autoware_planning_msgs::Trajectory &trajectory, const int closest) const;


  /* dynamic reconfigure */
  dynamic_reconfigure::Server<velocity_planner::VelocityPlannerConfig> dyncon_server_;
  
  void dynamicRecofCallback(velocity_planner::VelocityPlannerConfig &config, uint32_t level)
  {
    planning_param_.max_accel = config.max_accel;
    planning_param_.min_decel = config.min_decel;
    planning_param_.acc_jerk = config.max_acc_jerk;
    planning_param_.dec_jerk_nominal = config.min_dec_jerk_nominal;
    planning_param_.dec_jerk_urgent = config.min_dec_jerk_urgent;
    planning_param_.engage_velocity = config.engage_velocity;
    planning_param_.engage_acceleration = config.engage_acceleration;
    planning_param_.velocity_feedback_gain = config.velocity_feedback_gain;
  }


  class EmergencyStopManager
  {
  private:
    ros::Timer timer_;
    bool is_emergency_;
    bool is_counting_;
    ros::Time count_start_time_;
    const VelocityPlanner *velocity_planner_ptr_;
    double safety_time_for_stop_;

  public:
    EmergencyStopManager(const VelocityPlanner *velocity_planner_ptr, const double &safety_time_for_stop)
      : velocity_planner_ptr_(velocity_planner_ptr), safety_time_for_stop_(safety_time_for_stop)
    {
      timer_ = velocity_planner_ptr_->nh_.createTimer(ros::Duration(0.1),
                                                      &VelocityPlanner::EmergencyStopManager::timerCallback, this);
      is_emergency_ = false;
      is_counting_ = false;
    };
    void setEmergencyFlagTrue()
    {
      is_emergency_ = true;
      is_counting_ = false;
    };
    bool getEmergencyFlag()
    {
      return is_emergency_;
    };
    void countStart()
    {
      is_counting_ = true;
      count_start_time_ = ros::Time::now();
    };
    void countStop()
    {
      is_counting_ = false;
    };
    void timerCallback(const ros::TimerEvent &e)
    {
      if (velocity_planner_ptr_ == nullptr || velocity_planner_ptr_->current_velocity_ptr_ == nullptr)
        return;

      if (is_emergency_ == false)
        return;

      const double curr_v = std::fabs(velocity_planner_ptr_->current_velocity_ptr_->twist.linear.x);
      if (curr_v > 0.1)
      {
        ROS_WARN("EMERGENCY MANAGER : still moving");
        return;
      }
      if (is_counting_ == false)
      {
        ROS_WARN("EMERGENCY MANAGER : stop detected, start counting");
        countStart();
      }

      const ros::Time now = ros::Time::now();
      ROS_WARN("EMERGENCY MANAGER : now counting... %3.3f", (now - count_start_time_).toSec());
      if ((now - count_start_time_).toSec() > safety_time_for_stop_)
      {
        is_emergency_ = false;
        is_counting_ = false;
      }
    }
  };
  std::shared_ptr<EmergencyStopManager> emergency_stop_manager_ptr_;

  /* debug */
  ros::Publisher debug_closest_velocity_;
  ros::Publisher pub_debug_planning_jerk_;
  void publishClosestVelocity(const double &vel) const;
  void publishPlanningJerk(const double &jerk) const;

#ifdef USE_MATPLOTLIB_FOR_VELOCITY_VIZ
  void plotMotionVelocity(const std::string &color_str) const;
  void plotMotionAcceleration(const std::string &color_str) const;
  void plotMotionJerk(const std::string &color_str) const;
  void plotAll(const int &stop_idx_zero_vel, const int &input_closest, const autoware_planning_msgs::Trajectory &base,
               const autoware_planning_msgs::Trajectory &latacc_base, const autoware_planning_msgs::Trajectory &moveave_filtered,
               const autoware_planning_msgs::Trajectory &jerk_filtered, const autoware_planning_msgs::Trajectory &final) const;
#endif
};

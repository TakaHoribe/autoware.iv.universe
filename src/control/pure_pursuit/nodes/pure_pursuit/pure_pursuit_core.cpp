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

#include "pure_pursuit/pure_pursuit_core.h"
#include "pure_pursuit/pure_pursuit_viz.h"

namespace waypoint_follower {
// Constructor
PurePursuitNode::PurePursuitNode()
    : private_nh_("~"),
      ppdconf_ptr_(nullptr),
      curr_vel_ptr_(nullptr),
      curr_traj_ptr_(nullptr),
      curr_pose_ptr_(nullptr) {
  pp_ptr_ = std::unique_ptr<planning_utils::PurePursuit>(new planning_utils::PurePursuit());
  initForROS();
}

void PurePursuitNode::initForROS() {
  // ros parameter settings
  bool use_lerp = false;
  private_nh_.param<bool>("use_lerp", use_lerp, true);
  pp_ptr_->setUseLerp(use_lerp);
  private_nh_.param<bool>("publish_twist_cmd", pub_twist_cmd_, true);
  private_nh_.param<bool>("publish_ctrl_cmd", pub_ctrl_cmd_, true);
  nh_.param<double>("vehicle_info/wheel_base", wheel_base_, 2.7);
  private_nh_.param<double>("control_period", ctrl_period_, 0.02);
  private_nh_.param<double>("velocity_delay_compensation_time", velocity_delay_compensation_time_, 0.17);  // [s]
  private_nh_.param<bool>("use_lateral_error_compensation", use_lat_error_compensation_, false);
  private_nh_.param<double>("lateral_error_compensation_ratio", lec_ratio_, 1.0);
  private_nh_.param<double>("lateral_error_compensation_max", lec_max_, 2.0);
  private_nh_.param<bool>("use_steering_delay_compensation", use_steering_delay_compensation_, false);
  private_nh_.param<double>("steering_delay_compensation_time", steering_delay_compensation_time_, 0.5);

  // setup processing timer
  proc_timer_ = nh_.createTimer(ros::Duration(ctrl_period_), &PurePursuitNode::timerCallback, this);

  // setup subscriber
  traj_sub_ = nh_.subscribe("/control_trajectory", 1, &PurePursuitNode::trajCallback, this);
  pose_sub_ = nh_.subscribe("/current_pose", 1, &PurePursuitNode::cpCallback, this);
  config_sub_ = nh_.subscribe("/config/waypoint_follower", 1, &PurePursuitNode::configCallback, this);
  vel_sub_ = nh_.subscribe("/current_velocity", 1, &PurePursuitNode::cvCallback, this);

  // setup publisher
  twist_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/twist_cmd", 1);
  ctrl_pub_ = nh_.advertise<autoware_msgs::ControlCommandStamped>("/ctrl_cmd", 1);

  // publisher for debug
  viz_pub_ = private_nh_.advertise<visualization_msgs::MarkerArray>("debug/marker", 0);
  ld_pub_ = private_nh_.advertise<std_msgs::Float32>("debug/lookahead_distance", 0);
  le_pub_ = private_nh_.advertise<std_msgs::Float32>("debug/lateral_error", 0);
  cv_pub_ = private_nh_.advertise<std_msgs::Float32>("debug/current_command_velocity", 0);
  lec_pub_ = private_nh_.advertise<std_msgs::Float32>("debug/lateral_error_compensation", 0);
  sdc_pub_ = private_nh_.advertise<geometry_msgs::PoseStamped>("debug/steering_delay_compensation_pose", 0);
}

void PurePursuitNode::timerCallback(const ros::TimerEvent& event) {
  ROS_INFO("processing...");
  double diff_expected = event.current_expected.toSec() - event.last_expected.toSec();
  double diff_real = event.current_real.toSec() - event.last_real.toSec();
  ROS_DEBUG("diff_expected: %lf, diff_real: %lf", diff_expected, diff_real);

  auto proc_start = std::chrono::system_clock::now();

  if (curr_vel_ptr_ == nullptr || curr_traj_ptr_ == nullptr || curr_pose_ptr_ == nullptr || ppdconf_ptr_ == nullptr ||
      !pp_ptr_->isRequirementsSatisfied()) {
    ROS_WARN("Necessary topics are not subscribed yet ... ");
    return;
  }

  auto clst_pair = planning_utils::findClosestIdxWithDistAngThr(planning_utils::extractPoses(*curr_traj_ptr_),
                                                                curr_pose_ptr_->pose, 3.0, M_PI_4);

  ROS_DEBUG("curr_bool: %d, clst_idx: %d", clst_pair.first, clst_pair.second);

  if (!clst_pair.first) {
    ROS_WARN("cannot find closest waypoint");
    publishZeroCommand();
    return;
  }

  double cmd_vel = computeCommandVelocity(*ppdconf_ptr_, curr_traj_ptr_->motions, clst_pair.second);
  // double cmd_acc = computeCommandAcceleration(*ppdconf_ptr_, curr_traj_ptr_->motions, clst_pair.second);
  double cmd_acc =
      computeCommandAccelerationWithDelayCompensation(*ppdconf_ptr_, curr_traj_ptr_->motions, clst_pair.second,
                                                      velocity_delay_compensation_time_, curr_vel_ptr_->twist.linear.x);

  double ld = computeLookaheadDistance(*ppdconf_ptr_, curr_vel_ptr_->twist.linear.x, cmd_vel);

  // lateral error compensation
  const double lat_error =
      computeLateralError(curr_pose_ptr_->pose.position, curr_traj_ptr_->motions, clst_pair.second);
  if (use_lat_error_compensation_) {
    double ratio = (1.0 * lec_ratio_ + fabs(lat_error));
    ratio = ratio > lec_max_ ? lec_max_ : ratio;
    ld *= ratio;
    publishLateralErrorCompensation(ratio);
  }

  publishLookaheadDistance(ld);

  pp_ptr_->setLookaheadDistance(ld);
  auto pp_res = pp_ptr_->run();
  ROS_DEBUG("pp_res_bool: %d, pp_res_idx: %lf", pp_res.first, pp_res.second);

  if (pp_res.first)
    publishCommand(pp_res.second, cmd_vel, cmd_acc);
  else
    publishZeroCommand();

  // store angular_z
  if (use_steering_delay_compensation_) {
    double angular_z = cmd_vel * pp_res.second;
    angular_z_buffer_.push_back(angular_z);
    if (angular_z_buffer_.size() > steering_delay_compensation_time_ / ctrl_period_) angular_z_buffer_.pop_front();
  }

  auto proc_end = std::chrono::system_clock::now();
  double proc_elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(proc_end - proc_start).count();
  ROS_INFO("processing time : %lf [ms]", proc_elapsed * 1.0e-6);

  // for debug
  geometry_msgs::Point loc_next_wp = pp_ptr_->getLocationOfNextWaypoint();
  geometry_msgs::Point loc_next_tgt = pp_ptr_->getLocationOfNextTarget();
  publishVisualizer(loc_next_wp, loc_next_tgt, curr_pose_ptr_->pose, ld);
  publishLateralError(lat_error);
}

void PurePursuitNode::publishCommand(double kappa, double cmd_vel, double cmd_acc) {
  if (pub_twist_cmd_) publishTwistStamped(kappa, cmd_vel);

  if (pub_ctrl_cmd_) publishControlCommandStamped(kappa, cmd_vel, cmd_acc);

  publishCurrentCommandVelocity(cmd_vel);
}

void PurePursuitNode::publishZeroCommand() {
  if (pub_twist_cmd_) publishTwistStamped(0.0, 0.0);

  if (pub_ctrl_cmd_) publishControlCommandStamped(0.0, 0.0, 0.0);

  publishCurrentCommandVelocity(0.0);
}

void PurePursuitNode::publishTwistStamped(double kappa, double cmd_vel) {
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.twist.linear.x = cmd_vel;
  ts.twist.angular.z = kappa * ts.twist.linear.x;
  twist_pub_.publish(ts);
}

void PurePursuitNode::publishControlCommandStamped(double kappa, double cmd_vel, double cmd_acc) {
  autoware_msgs::ControlCommandStamped ccs;
  ccs.header.stamp = ros::Time::now();
  ccs.cmd.linear_velocity = cmd_vel;
  ccs.cmd.linear_acceleration = cmd_acc;
  ccs.cmd.steering_angle = planning_utils::convertCurvatureToSteeringAngle(wheel_base_, kappa);
  ctrl_pub_.publish(ccs);
}

void PurePursuitNode::publishCurrentCommandVelocity(double cmd_vel) {
  std_msgs::Float32 cv;
  cv.data = cmd_vel * 3.6;
  cv_pub_.publish(cv);
}

void PurePursuitNode::publishLateralError(double lat_error) {
  std_msgs::Float32 le;
  le.data = lat_error;

  le_pub_.publish(le);
}

void PurePursuitNode::publishLateralErrorCompensation(double lat_error_comp_ratio) {
  std_msgs::Float32 msg;
  msg.data = lat_error_comp_ratio;

  lec_pub_.publish(msg);
}

void PurePursuitNode::publishVisualizer(const geometry_msgs::Point& next_wp_pos,
                                        const geometry_msgs::Point& next_tgt_pos, const geometry_msgs::Pose& curr_pose,
                                        double ld) {
  visualization_msgs::MarkerArray viz_ma;
  viz_ma.markers.push_back(*displayNextWaypoint(next_wp_pos));
  viz_ma.markers.push_back(*displaySearchRadius(curr_pose.position, ld));
  viz_ma.markers.push_back(*displayNextTarget(next_tgt_pos));
  viz_ma.markers.push_back(*displayTrajectoryCircle(generateTrajectoryCircle(next_tgt_pos, curr_pose)));
  // auto traj = displayControlTrajectory(*curr_traj_ptr_);
  // viz_ma.markers.insert(viz_ma.markers.end(), traj.markers.begin(),traj.markers.end());
  viz_pub_.publish(viz_ma);

  ROS_DEBUG("next wp pos: %lf, %lf, %lf", next_wp_pos.x, next_wp_pos.y, next_wp_pos.z);
}

void PurePursuitNode::publishLookaheadDistance(double ld) {
  std_msgs::Float32 ld_msg;
  ld_msg.data = ld;
  ld_pub_.publish(ld_msg);
}

void PurePursuitNode::configCallback(const autoware_config_msgs::ConfigWaypointFollowerConstPtr& config) {
  ROS_DEBUG_STREAM(__func__);
  ppdconf_ptr_ = std::unique_ptr<PurePursuitDynamicConfig>(new PurePursuitDynamicConfig());
  ppdconf_ptr_->param_flag_ = config->param_flag;
  ppdconf_ptr_->const_lookahead_distance_ = config->lookahead_distance;
  ppdconf_ptr_->const_velocity_ = config->velocity;
  ppdconf_ptr_->lookahead_distance_ratio_ = config->lookahead_ratio;
  ppdconf_ptr_->minimum_lookahead_distance_ = config->minimum_lookahead_distance;
  private_nh_.param<double>("reverse_minimum_lookahead_distance", ppdconf_ptr_->reverse_minld_, 6.0);
}

void PurePursuitNode::cpCallback(const geometry_msgs::PoseStampedConstPtr& msg) {
  ROS_DEBUG_STREAM(__func__);
  curr_pose_ptr_ = msg;
  pp_ptr_->setCurrentPose(msg->pose);

  if (use_steering_delay_compensation_ && curr_vel_ptr_ != nullptr) {
    // control delay compensation
    const auto& sdc_pose = computePoseWithSteeringDelayCompensation(curr_pose_ptr_->pose, curr_vel_ptr_->twist.linear.x,
                                                                    angular_z_buffer_, ctrl_period_);
    geometry_msgs::PoseStamped sdc_pose_s;
    sdc_pose_s.header.stamp = ros::Time::now();
    sdc_pose_s.header.frame_id = "map";
    sdc_pose_s.pose = sdc_pose;
    sdc_pub_.publish(sdc_pose_s);
    pp_ptr_->setCurrentPose(sdc_pose);
  }
}

void PurePursuitNode::cvCallback(const geometry_msgs::TwistStampedConstPtr& msg) {
  ROS_DEBUG_STREAM(__func__);
  curr_vel_ptr_ = msg;
}

void PurePursuitNode::trajCallback(const autoware_planner_msgs::TrajectoryConstPtr& msg) {
  ROS_DEBUG_STREAM(__func__);
  curr_traj_ptr_ = msg;

  pp_ptr_->setWaypoints(planning_utils::extractPoses(*msg));
}

double computeLookaheadDistance(const PurePursuitDynamicConfig& ppdconf, double curr_linear_vel, double cmd_vel) {
  if (ppdconf.param_flag_ == enumToInteger(Mode::dialog)) return ppdconf.const_lookahead_distance_;

  double ld = fabs(curr_linear_vel) * ppdconf.lookahead_distance_ratio_;
  double minld = (cmd_vel < 0) ? ppdconf.reverse_minld_ : ppdconf.minimum_lookahead_distance_;

  return (ld > minld) ? ld : minld;
}

double computeCommandVelocity(const PurePursuitDynamicConfig& ppdconf,
                              const std::vector<autoware_planner_msgs::MotionStamped>& curr_mos, int32_t clst_wp_idx) {
  double cmd_vel = curr_mos.at(clst_wp_idx).motion.twist.linear.x;
  if (ppdconf.param_flag_ == enumToInteger(Mode::dialog)) {
    const int sgn = (cmd_vel < 0) ? -1 : 1;
    cmd_vel = sgn * planning_utils::kmph2mps(ppdconf.const_velocity_);
  }

  ROS_DEBUG("cmd_vel: %lf", cmd_vel);
  return cmd_vel;
}

double computeCommandAcceleration(const PurePursuitDynamicConfig& ppdconf,
                                  const std::vector<autoware_planner_msgs::MotionStamped>& curr_mos,
                                  int32_t clst_wp_idx) {
  double cmd_acc = curr_mos.at(clst_wp_idx).motion.accel.linear.x;
  if (ppdconf.param_flag_ == enumToInteger(Mode::dialog)) cmd_acc = 0.0;

  ROS_DEBUG("amd_acc: %lf", cmd_acc);
  return cmd_acc;
}

double computeCommandAccelerationWithDelayCompensation(
    const PurePursuitDynamicConfig& ppdconf, const std::vector<autoware_planner_msgs::MotionStamped>& curr_mos,
    const int32_t clst_wp_idx, const double& delay, const double& cur_v) {
  const double delay_dist = cur_v * delay;  // [m]

  int32_t idx = clst_wp_idx;
  double dist_sum = 0.0;
  for (unsigned int i = idx; i < curr_mos.size() - 1; ++i) {
    idx = i;
    dist_sum +=
        planning_utils::calcDistance2D(curr_mos.at(i).motion.pose.position, curr_mos.at(i + 1).motion.pose.position);
    if (dist_sum > delay_dist) break;
  }

  double cmd_acc = curr_mos.at(idx).motion.accel.linear.x;
  if (ppdconf.param_flag_ == enumToInteger(Mode::dialog)) cmd_acc = 0.0;

  ROS_DEBUG("amd_acc: %lf", cmd_acc);
  return cmd_acc;
}

double computeLateralError(const geometry_msgs::Point& point,
                           const std::vector<autoware_planner_msgs::MotionStamped>& curr_mos, int32_t clst_wp_idx) {
  // Calculate the deviation of current position from the waypoint approximate
  // line

  double lat_error;

  if (curr_mos.size() < 2) return 0.0;

  if (clst_wp_idx == 0)
    lat_error = planning_utils::calcLateralError2D(curr_mos.at(clst_wp_idx).motion.pose.position,
                                                   curr_mos.at(clst_wp_idx + 1).motion.pose.position, point);
  else
    lat_error = planning_utils::calcLateralError2D(curr_mos.at(clst_wp_idx - 1).motion.pose.position,
                                                   curr_mos.at(clst_wp_idx).motion.pose.position, point);

  return lat_error;
}

geometry_msgs::Pose computePoseWithSteeringDelayCompensation(const geometry_msgs::Pose& curr_pose, double cmd_linear_x,
                                                             const std::deque<double>& angular_z_buf,
                                                             double time_delta) {
  geometry_msgs::Pose res_pose = curr_pose;
  for (const auto& e : angular_z_buf) {
    res_pose.position.x += cmd_linear_x * cos(tf2::getYaw(res_pose.orientation)) * time_delta;
    res_pose.position.y += cmd_linear_x * sin(tf2::getYaw(res_pose.orientation)) * time_delta;
    tf2::Quaternion add_q;
    add_q.setRPY(0.0, 0.0, e * time_delta);
    tf2::Quaternion q;
    tf2::fromMsg(res_pose.orientation, q);
    tf2::Quaternion res_q = q * add_q;
    res_q.normalize();
    tf2::convert(res_q, res_pose.orientation);
  }

  return res_pose;
}
}  // namespace waypoint_follower

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

PurePursuitNode::PurePursuitNode() : nh_(""), private_nh_("~"), tf_listener_(tf_buffer_) {
  pp_ptr_ = std::unique_ptr<planning_utils::PurePursuit>(new planning_utils::PurePursuit());

  // Parameter
  private_nh_.param<bool>("use_lerp", use_lerp_, true);
  private_nh_.param<bool>("publish_twist_cmd", publish_twist_cmd_, true);
  private_nh_.param<bool>("publish_ctrl_cmd", publish_ctrl_cmd_, true);
  private_nh_.param<double>("control_period", ctrl_period_, 0.02);
  private_nh_.param<double>("velocity_delay_compensation_time", velocity_delay_compensation_time_, 0.17);  // [s]
  private_nh_.param<bool>("use_lateral_error_compensation", use_lat_error_compensation_, false);
  private_nh_.param<double>("lateral_error_compensation_ratio", lec_ratio_, 1.0);
  private_nh_.param<double>("lateral_error_compensation_max", lec_max_, 2.0);
  private_nh_.param<bool>("use_steering_delay_compensation", use_steering_delay_compensation_, false);
  private_nh_.param<double>("steering_delay_compensation_time", steering_delay_compensation_time_, 0.5);

  private_nh_.param<double>("/vehicle_info/wheel_base", wheel_base_, 2.7);

  // Subscribers
  sub_trajectory_ = private_nh_.subscribe("input/reference_trajectory", 1, &PurePursuitNode::onTrajectory, this);
  sub_current_velocity_ = private_nh_.subscribe("input/current_velocity", 1, &PurePursuitNode::onCurrentVelocity, this);

  // Publishers
  pub_twist_cmd_ = private_nh_.advertise<geometry_msgs::TwistStamped>("output/twist_raw", 1);
  pub_ctrl_cmd_ = private_nh_.advertise<autoware_control_msgs::ControlCommandStamped>("output/control_raw", 1);

  // Debug Publishers
  pub_viz_ = private_nh_.advertise<visualization_msgs::MarkerArray>("debug/marker", 0);
  pub_ld_ = private_nh_.advertise<std_msgs::Float32>("debug/lookahead_distance", 0);
  pub_le_ = private_nh_.advertise<std_msgs::Float32>("debug/lateral_error", 0);
  pub_cv_ = private_nh_.advertise<std_msgs::Float32>("debug/current_command_velocity", 0);
  pub_lec_ = private_nh_.advertise<std_msgs::Float32>("debug/lateral_error_compensation", 0);
  pub_sdc_ = private_nh_.advertise<geometry_msgs::PoseStamped>("debug/steering_delay_compensation_pose", 0);

  // Timer
  timer_ = nh_.createTimer(ros::Duration(ctrl_period_), &PurePursuitNode::onTimer, this);

  // Setup
  pp_ptr_->setUseLerp(use_lerp_);

  //  Wait for first current pose
  while (ros::ok()) {
    try {
      tf_buffer_.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(5.0));
      break;
    } catch (tf2::TransformException& ex) {
      ROS_INFO("waiting for current pose...");
    }
  }

  // tmp: config topic
  // TODO: parameterize
  ppdconf_ptr_ = std::unique_ptr<PurePursuitDynamicConfig>(new PurePursuitDynamicConfig());
  ppdconf_ptr_->param_flag_ = 0;
  ppdconf_ptr_->const_velocity_ = 1.0;
  ppdconf_ptr_->const_lookahead_distance_ = 4;
  ppdconf_ptr_->lookahead_distance_ratio_ = 2.2;
  ppdconf_ptr_->minimum_lookahead_distance_ = 2.5;
  private_nh_.param<double>("reverse_minimum_lookahead_distance", ppdconf_ptr_->reverse_minld_, 6.0);
}

void PurePursuitNode::onTimer(const ros::TimerEvent& event) {
  updateCurrentPose();

  pp_ptr_->setCurrentPose(current_pose_ptr_->pose);

  if (use_steering_delay_compensation_ && current_velocity_ptr_ != nullptr) {
    // control delay compensation
    const auto& sdc_pose = computePoseWithSteeringDelayCompensation(
        current_pose_ptr_->pose, current_velocity_ptr_->twist.linear.x, angular_z_buffer_, ctrl_period_);
    geometry_msgs::PoseStamped sdc_pose_s;
    sdc_pose_s.header.stamp = ros::Time::now();
    sdc_pose_s.header.frame_id = "map";
    sdc_pose_s.pose = sdc_pose;
    pub_sdc_.publish(sdc_pose_s);
    pp_ptr_->setCurrentPose(sdc_pose);
  }

  const double diff_expected = event.current_expected.toSec() - event.last_expected.toSec();
  const double diff_real = event.current_real.toSec() - event.last_real.toSec();

  if (!current_velocity_ptr_ || !current_trajectory_ptr_ || !current_pose_ptr_ || !ppdconf_ptr_ ||
      !pp_ptr_->isRequirementsSatisfied()) {
    ROS_WARN_THROTTLE(5.0, "Necessary topics are not subscribed yet ... ");
    return;
  }

  auto clst_pair = planning_utils::findClosestIdxWithDistAngThr(planning_utils::extractPoses(*current_trajectory_ptr_),
                                                                current_pose_ptr_->pose, 3.0, M_PI_4);

  ROS_DEBUG("curr_bool: %d, clst_idx: %d", clst_pair.first, clst_pair.second);

  if (!clst_pair.first) {
    ROS_WARN("cannot find closest waypoint");
    publishZeroCommand();
    return;
  }

  const double cmd_vel = computeCommandVelocityWithDelayCompensation(
      *ppdconf_ptr_, current_trajectory_ptr_->points, clst_pair.second, velocity_delay_compensation_time_,
      current_velocity_ptr_->twist.linear.x);
  const double cmd_acc = computeCommandAccelerationWithDelayCompensation(
      *ppdconf_ptr_, current_trajectory_ptr_->points, clst_pair.second, velocity_delay_compensation_time_,
      current_velocity_ptr_->twist.linear.x);

  double ld = computeLookaheadDistance(*ppdconf_ptr_, current_velocity_ptr_->twist.linear.x, cmd_vel);

  // lateral error compensation
  const double lat_error =
      computeLateralError(current_pose_ptr_->pose.position, current_trajectory_ptr_->points, clst_pair.second);
  if (use_lat_error_compensation_) {
    double ratio = (1.0 * lec_ratio_ + fabs(lat_error));
    ratio = ratio > lec_max_ ? lec_max_ : ratio;
    ld *= ratio;
    publishLateralErrorCompensation(ratio);
  }

  publishLookaheadDistance(ld);

  pp_ptr_->setLookaheadDistance(ld);
  const auto pp_res = pp_ptr_->run();
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

  // for debug
  geometry_msgs::Point loc_next_wp = pp_ptr_->getLocationOfNextWaypoint();
  geometry_msgs::Point loc_next_tgt = pp_ptr_->getLocationOfNextTarget();
  publishVisualizer(loc_next_wp, loc_next_tgt, current_pose_ptr_->pose, ld);
  publishLateralError(lat_error);
}

void PurePursuitNode::publishCommand(double kappa, double cmd_vel, double cmd_acc) {
  if (publish_twist_cmd_) publishTwistStamped(kappa, cmd_vel);

  if (publish_ctrl_cmd_) publishControlCommandStamped(kappa, cmd_vel, cmd_acc);

  publishCurrentCommandVelocity(cmd_vel);
}

void PurePursuitNode::publishZeroCommand() {
  if (publish_twist_cmd_) publishTwistStamped(0.0, 0.0);

  if (publish_ctrl_cmd_) publishControlCommandStamped(0.0, 0.0, 0.0);

  publishCurrentCommandVelocity(0.0);
}

void PurePursuitNode::publishTwistStamped(double kappa, double cmd_vel) {
  geometry_msgs::TwistStamped ts;
  ts.header.stamp = ros::Time::now();
  ts.twist.linear.x = cmd_vel;
  ts.twist.angular.z = kappa * ts.twist.linear.x;
  pub_twist_cmd_.publish(ts);
}

void PurePursuitNode::publishControlCommandStamped(double kappa, double cmd_vel, double cmd_acc) {
  autoware_control_msgs::ControlCommandStamped ccs;
  ccs.header.stamp = ros::Time::now();
  ccs.control.velocity = cmd_vel;
  ccs.control.acceleration = cmd_acc;
  ccs.control.steering_angle = planning_utils::convertCurvatureToSteeringAngle(wheel_base_, kappa);
  pub_ctrl_cmd_.publish(ccs);
}

void PurePursuitNode::publishCurrentCommandVelocity(double cmd_vel) {
  std_msgs::Float32 cv;
  cv.data = cmd_vel * 3.6;
  pub_cv_.publish(cv);
}

void PurePursuitNode::publishLateralError(double lat_error) {
  std_msgs::Float32 le;
  le.data = lat_error;
  pub_le_.publish(le);
}

void PurePursuitNode::publishLateralErrorCompensation(double lat_error_comp_ratio) {
  std_msgs::Float32 msg;
  msg.data = lat_error_comp_ratio;
  pub_lec_.publish(msg);
}

void PurePursuitNode::publishVisualizer(const geometry_msgs::Point& next_wp_pos,
                                        const geometry_msgs::Point& next_tgt_pos, const geometry_msgs::Pose& curr_pose,
                                        double ld) {
  visualization_msgs::MarkerArray viz_ma;
  viz_ma.markers.push_back(*displayNextWaypoint(next_wp_pos));
  viz_ma.markers.push_back(*displaySearchRadius(curr_pose.position, ld));
  viz_ma.markers.push_back(*displayNextTarget(next_tgt_pos));
  viz_ma.markers.push_back(*displayTrajectoryCircle(generateTrajectoryCircle(next_tgt_pos, curr_pose)));
  pub_viz_.publish(viz_ma);
}

void PurePursuitNode::publishLookaheadDistance(double ld) {
  std_msgs::Float32 ld_msg;
  ld_msg.data = ld;
  pub_ld_.publish(ld_msg);
}

void PurePursuitNode::updateCurrentPose() {
  geometry_msgs::TransformStamped transform;
  try {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  } catch (tf2::TransformException& ex) {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[mpc_follower] cannot get map to base_link transform. %s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(ps);
}

void PurePursuitNode::onCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  current_velocity_ptr_ = msg;
}

void PurePursuitNode::onTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr& msg) {
  current_trajectory_ptr_ = msg;

  pp_ptr_->setWaypoints(planning_utils::extractPoses(*msg));
}

double computeLookaheadDistance(const PurePursuitDynamicConfig& ppdconf, double curr_linear_vel, double cmd_vel) {
  if (ppdconf.param_flag_ == enumToInteger(Mode::dialog)) return ppdconf.const_lookahead_distance_;

  double ld = fabs(curr_linear_vel) * ppdconf.lookahead_distance_ratio_;
  double minld = (cmd_vel < 0) ? ppdconf.reverse_minld_ : ppdconf.minimum_lookahead_distance_;

  return (ld > minld) ? ld : minld;
}

double computeCommandVelocity(const PurePursuitDynamicConfig& ppdconf,
                              const std::vector<autoware_planning_msgs::TrajectoryPoint>& points, int32_t clst_wp_idx) {
  double cmd_vel = points.at(clst_wp_idx).twist.linear.x;
  if (ppdconf.param_flag_ == enumToInteger(Mode::dialog)) {
    const int sgn = (cmd_vel < 0) ? -1 : 1;
    cmd_vel = sgn * planning_utils::kmph2mps(ppdconf.const_velocity_);
  }

  ROS_DEBUG("cmd_vel: %lf", cmd_vel);
  return cmd_vel;
}

double computeCommandAcceleration(const PurePursuitDynamicConfig& ppdconf,
                                  const std::vector<autoware_planning_msgs::TrajectoryPoint>& points,
                                  int32_t clst_wp_idx) {
  double cmd_acc = points.at(clst_wp_idx).accel.linear.x;
  if (ppdconf.param_flag_ == enumToInteger(Mode::dialog)) cmd_acc = 0.0;

  ROS_DEBUG("amd_acc: %lf", cmd_acc);
  return cmd_acc;
}

autoware_planning_msgs::TrajectoryPoint computeTargetPointsWithDelayCompensation(
    const PurePursuitDynamicConfig& ppdconf, const std::vector<autoware_planning_msgs::TrajectoryPoint>& points,
    const int32_t clst_wp_idx, const double& delay, const double& curr_vel) {
  const double delay_dist = curr_vel * delay;  // [m]

  int32_t idx = clst_wp_idx;
  double dist_sum = 0.0;
  for (unsigned int i = idx; i < points.size() - 1; ++i) {
    idx = i;
    dist_sum += planning_utils::calcDistance2D(points.at(i).pose.position, points.at(i + 1).pose.position);
    if (dist_sum > delay_dist) break;
  }

  return points.at(idx);
}

double computeCommandVelocityWithDelayCompensation(const PurePursuitDynamicConfig& ppdconf,
                                                   const std::vector<autoware_planning_msgs::TrajectoryPoint>& points,
                                                   const int32_t clst_wp_idx, const double& delay,
                                                   const double& curr_vel) {
  auto target_point = computeTargetPointsWithDelayCompensation(ppdconf, points, clst_wp_idx, delay, curr_vel);
  double cmd_vel = target_point.twist.linear.x;

  if (ppdconf.param_flag_ == enumToInteger(Mode::dialog)) {
    const int sgn = (cmd_vel < 0) ? -1 : 1;
    cmd_vel = sgn * planning_utils::kmph2mps(ppdconf.const_velocity_);
  }

  ROS_DEBUG("cmd_vel: %lf", cmd_vel);
  return cmd_vel;
}

double computeCommandAccelerationWithDelayCompensation(
    const PurePursuitDynamicConfig& ppdconf, const std::vector<autoware_planning_msgs::TrajectoryPoint>& points,
    const int32_t clst_wp_idx, const double& delay, const double& curr_vel) {
  auto target_point = computeTargetPointsWithDelayCompensation(ppdconf, points, clst_wp_idx, delay, curr_vel);
  double cmd_acc = target_point.accel.linear.x;
  if (ppdconf.param_flag_ == enumToInteger(Mode::dialog)) cmd_acc = 0.0;

  ROS_DEBUG("amd_acc: %lf", cmd_acc);
  return cmd_acc;
}

double computeLateralError(const geometry_msgs::Point& point,
                           const std::vector<autoware_planning_msgs::TrajectoryPoint>& points, int32_t clst_wp_idx) {
  // Calculate the deviation of current position from the waypoint approximate line

  double lat_error;

  if (points.size() < 2) return 0.0;

  if (clst_wp_idx == 0)
    lat_error = planning_utils::calcLateralError2D(points.at(clst_wp_idx).pose.position,
                                                   points.at(clst_wp_idx + 1).pose.position, point);
  else
    lat_error = planning_utils::calcLateralError2D(points.at(clst_wp_idx - 1).pose.position,
                                                   points.at(clst_wp_idx).pose.position, point);

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

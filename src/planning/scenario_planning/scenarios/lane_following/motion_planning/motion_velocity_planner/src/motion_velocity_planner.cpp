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

#include <motion_velocity_planner/motion_velocity_planner.hpp>
#include <chrono>

#define DEBUG_INFO(...) { if (show_debug_info_) {ROS_INFO(__VA_ARGS__); } }
#define DEBUG_WARN(...) { if (show_debug_info_) {ROS_WARN(__VA_ARGS__); } }
#define DEBUG_INFO_ALL(...) { if (show_debug_info_all_) {ROS_INFO(__VA_ARGS__); } }

MotionVelocityPlanner::MotionVelocityPlanner() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.param("max_velocity", planning_param_.max_velocity, double(11.11));              // 40.0 kmph
  pnh_.param("max_accel", planning_param_.max_accel, double(1.0));                         // 0.11G
  pnh_.param("min_decel", planning_param_.min_decel, double(-2.0));                        // -0.2G
  pnh_.param("max_acc_jerk", planning_param_.acc_jerk, double(0.3));                   // 0.03G
  pnh_.param("min_dec_jerk_nominal", planning_param_.dec_jerk_nominal, double(-0.3));  // -0.03G
  pnh_.param("min_dec_jerk_urgent", planning_param_.dec_jerk_urgent, double(-1.5));          //
  pnh_.param("large_jerk_report", planning_param_.large_jerk_report, double(-1.4));          //
  pnh_.param("max_lateral_accel", planning_param_.max_lateral_accel, double(0.2));          //
  pnh_.param("replan_vel_deviation", planning_param_.replan_vel_deviation, double(3.0));
  pnh_.param("engage_velocity", planning_param_.engage_velocity, double(0.3));
  pnh_.param("engage_acceleration", planning_param_.engage_acceleration, double(0.1));
  pnh_.param("stopping_speed", planning_param_.stopping_speed, double(0.0));
  pnh_.param("extract_ahead_dist", planning_param_.extract_ahead_dist, double(30.0));
  pnh_.param("extract_behind_dist", planning_param_.extract_behind_dist, double(2.0));
  pnh_.param("resample_num", planning_param_.resample_num, int(10));
  pnh_.param("replan_stop_point_change_dist", planning_param_.replan_stop_point_change_dist, double(2.0));
  pnh_.param("velocity_feedback_gain", planning_param_.velocity_feedback_gain, double(0.3));
  pnh_.param("stop_dist_to_prohibit_engage", planning_param_.stop_dist_to_prohibit_engage, double(1.5));
  pnh_.param("emergency_flag_vel_thr_kmph", planning_param_.emergency_flag_vel_thr_kmph, double(3.0));
  pnh_.param("jerk_planning_span", planning_param_.jerk_planning_span, double(0.1));
  pnh_.param("stop_dist_mergin", planning_param_.stop_dist_mergin, double(0.55));
  
  pnh_.param("enable_to_publish_emergency", enable_to_publish_emergency_, bool(true));

  pnh_.param("show_debug_info", show_debug_info_, bool(true));
  pnh_.param("show_debug_info_all", show_debug_info_all_, bool(false));
  pnh_.param("show_figure", show_figure_, bool(false));
  pnh_.param("enable_latacc_filter", enable_latacc_filter_, bool(false));


  timer_replan_ = nh_.createTimer(ros::Duration(0.1), &MotionVelocityPlanner::timerReplanCallback, this);
  pub_trajectory_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);
  pub_is_emergency_ = pnh_.advertise<std_msgs::Bool>("is_sudden_stop", 1);
  pub_dist_to_stopline_ = pnh_.advertise<std_msgs::Float32>("distance_to_stopline", 1);
  sub_current_trajectory_ = pnh_.subscribe("input/trajectory", 1, &MotionVelocityPlanner::callbackCurrentTrajectory, this);
  sub_current_velocity_ = pnh_.subscribe("/vehicle/status/twist", 1, &MotionVelocityPlanner::callbackCurrentVelocity, this);
  sub_external_velocity_limit_ = pnh_.subscribe("/external_velocity_limit_mps", 1, &MotionVelocityPlanner::callbackExternalVelocityLimit, this);

  /* for emergency stop manager */
  double emergency_stop_time;
  pnh_.param("emergency_stop_time", emergency_stop_time, double());
  emergency_stop_manager_ptr_ = std::make_shared<EmergencyStopManager>(this, emergency_stop_time);

  prev_stop_planning_jerk_ = 0.0;

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<motion_velocity_planner::MotionVelocityPlannerConfig>::CallbackType dyncon_f =
      boost::bind(&MotionVelocityPlanner::dynamicRecofCallback, this, _1, _2);
  dyncon_server_.setCallback(dyncon_f);

  /* debug */
  debug_closest_velocity_ = pnh_.advertise<std_msgs::Float32>("closest_velocity", 1);
  pub_debug_planning_jerk_ = pnh_.advertise<std_msgs::Float32>("current_planning_jerk", 1);

  /* wait to get vehicle position */
  while (ros::ok())
  {
    try
    {
      tf_buffer_.lookupTransform("map", "base_link", ros::Time::now(), ros::Duration(5.0));
      break;
    }
    catch (tf2::TransformException &ex)
    {
      ROS_INFO("[MotionVelocityPlanner] is waitting to get map to base_link transform. %s", ex.what());
      continue;
    }
  }
};
MotionVelocityPlanner::~MotionVelocityPlanner(){};

void MotionVelocityPlanner::publishTrajectory(const autoware_planning_msgs::Trajectory &trajectory) const
{
  pub_trajectory_.publish(trajectory);
}

void MotionVelocityPlanner::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr msg)
{
  current_velocity_ptr_ = msg;
}
void MotionVelocityPlanner::callbackCurrentTrajectory(const autoware_planning_msgs::Trajectory::ConstPtr msg)
{
  base_traj_raw_ptr_ = msg;
}
void MotionVelocityPlanner::callbackExternalVelocityLimit(const std_msgs::Float32::ConstPtr msg)
{
  external_velocity_limit_ptr_ = msg;
}

void MotionVelocityPlanner::updateCurrentPose()
{

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(
        "map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("[MotionVelocityPlanner] cannot get map to base_link transform. %s", ex.what());
    return;
  }

  geometry_msgs::PoseStamped ps;
  ps.header = transform.header;
  ps.pose.position.x = transform.transform.translation.x;
  ps.pose.position.y = transform.transform.translation.y;
  ps.pose.position.z = transform.transform.translation.z;
  ps.pose.orientation = transform.transform.rotation;
  current_pose_ptr_ = boost::make_shared<geometry_msgs::PoseStamped>(ps);
};

void MotionVelocityPlanner::timerReplanCallback(const ros::TimerEvent &e)
{
  updateCurrentPose();

  auto t_start = std::chrono::system_clock::now();

  DEBUG_INFO("============================== timer callback start ==============================");

  /* (0) guard */
  if (current_pose_ptr_ == nullptr || current_velocity_ptr_ == nullptr || base_traj_raw_ptr_ == nullptr)
  {
    DEBUG_INFO("wait topics : current_pose = %d, current_velocity = %d, base_waypoints = %d", (bool)current_pose_ptr_,
               (bool)current_velocity_ptr_, (bool)base_traj_raw_ptr_);
    return;
  }
  if (base_traj_raw_ptr_->points.size() == 0)
  {
    DEBUG_INFO("empty base_waypoint, return. : base_traj.size = %lu", base_traj_raw_ptr_->points.size());
    return;
  }

  /* (1) Find the nearest point to base_waypoints. This is not severe so, could read closest topic by callback. */
  int base_raw_closest = vpu::calcClosestWaypoint(*base_traj_raw_ptr_, current_pose_ptr_->pose);
  if (base_raw_closest < 0)
  {
    ROS_WARN("[velocity planner] cannot find closest waypoint for base raw trajectory");
    return;
  }

  /* (2) extruct the route surrounding the self-position from base_waypoints */
  autoware_planning_msgs::Trajectory base_traj_extracted;
  if (!vpu::extractPathAroundIndex(*base_traj_raw_ptr_, base_raw_closest, planning_param_.extract_ahead_dist,
                                   planning_param_.extract_behind_dist, /* out */ base_traj_extracted))
  {
    ROS_WARN("[motion_velocity_planner] extractPathAroundIndex failed. base_traj_raw_ptr_->size() = %lu, "
             "base_raw_closest = %d, extract_ahead_dist = %f, extract_behind_dist = %f",
             base_traj_raw_ptr_->points.size(), base_raw_closest, planning_param_.extract_ahead_dist,
             planning_param_.extract_behind_dist);
    return;
  }
  DEBUG_INFO("[extractPathAroundIndex] : base_raw.size() = %lu, base_closest = %d, "
             "base_raw_extracted.size() = %lu",
             base_traj_raw_ptr_->points.size(), base_raw_closest, base_traj_extracted.points.size());

  /* (3) Apply external velocity limit */
  if (external_velocity_limit_ptr_ != nullptr)
  {
    DEBUG_INFO("[motion_velocity_planner] : apply external velocity lim : %3.3f, base_closest_vel = %3.3f",
               external_velocity_limit_ptr_->data,
               base_traj_raw_ptr_->points.at(base_raw_closest).twist.linear.x);
    vpu::maximumVelocityFilter(external_velocity_limit_ptr_->data, base_traj_extracted);
  }


  /* (4) Resample extructed-waypoints with interpolation */
  // autoware_planning_msgs::Trajectory base_traj_resampled;
  // if (!vpu::linearInterpPath(base_traj_extracted, planning_param_.resample_num, /* out */ base_traj_resampled))
  // {
  //   ROS_WARN("[motion_velocity_planner] linearInterpPath failed. base_traj_extracted.size = %lu, resample_num = %d",
  //            base_traj_extracted.points.size(), planning_param_.resample_num);
  // }
  // int base_traj_resampled_closest = vpu::calcClosestWaypoint(base_traj_resampled, current_pose_ptr_->pose);
  // if (base_traj_resampled_closest < 0)
  // {
  //   ROS_WARN("[motion_velocity_planner] cannot find closest idx for resampled trajectory");
  //   return;
  // }
  // DEBUG_INFO("[linearInterpPath] : base_resampled.size() = %lu, base_resampled_closest = %d",
  //            base_traj_resampled.points.size(), base_traj_resampled_closest);

  autoware_planning_msgs::Trajectory base_traj_resampled = base_traj_extracted;
  int base_traj_resampled_closest = vpu::calcClosestWaypoint(base_traj_resampled, current_pose_ptr_->pose);



  /* publish stop distance */
  publishStopDistance(base_traj_resampled, base_traj_resampled_closest);

  /* Change base velocity to zero when current_velocity == 0 & stop_dist is close */
  preventMoveToVeryCloseStopLine(base_traj_resampled_closest, base_traj_resampled);

  /* for negative velocity */
  const bool negative_velocity_flag =
      base_traj_resampled.points.at(base_traj_resampled_closest).twist.linear.x < 0.0 ? true : false;
  if (negative_velocity_flag == true)
  {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, /* out */ base_traj_resampled);
  }

  /* (5) Calculate the nearest point on the previously planned route (used to get initial planning speed) */
  int replanned_traj_closest = -1;
  if (replanned_traj_.points.size() != 0 /* if this is not initial planning */)
  {
    replanned_traj_closest = vpu::calcClosestWaypoint(replanned_traj_, current_pose_ptr_->pose);
  }
  DEBUG_INFO("[calcClosestWaypoint] for base_resampled : base_resampled.size() = %d, prev_planned_closest_ = %d",
             (int)base_traj_resampled.points.size(), replanned_traj_closest);

  /* (6) Replan velocity */
  const std::vector<Motion> prev_replanned_traj_motion = replanned_traj_motion_;
  double stop_planning_jerk;
  replanVelocity(base_traj_resampled, base_traj_resampled_closest, prev_replanned_traj_motion, replanned_traj_closest,
                /* out */ replanned_traj_, /* out */ replanned_traj_motion_, /* out */ stop_planning_jerk);
  prev_stop_planning_jerk_ = stop_planning_jerk;
  DEBUG_INFO("[replanVelocity] : current_replanned.size() = %d", (int)replanned_traj_.points.size());


  /* (7) max velocity filter for safety */
  vpu::maximumVelocityFilter(planning_param_.max_velocity, replanned_traj_);

  /* for negative velocity */
  if (negative_velocity_flag == true)
  {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, replanned_traj_);
  }

  /* publish message */
  replanned_traj_.header = base_traj_raw_ptr_->header;
  publishTrajectory(replanned_traj_);


  /* for debug */
  MotionVelocityPlanner::publishClosestVelocity(replanned_traj_.points.at(base_traj_resampled_closest).twist.linear.x);

  auto t_end = std::chrono::system_clock::now();
  double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() * 1.0e-6;
  DEBUG_INFO("timer callback: calculation time = %f [ms]", elapsed_ms);
  DEBUG_INFO("============================== timer callback end ==============================\n\n");
}

void MotionVelocityPlanner::publishStopDistance(const autoware_planning_msgs::Trajectory &trajectory, const int closest) const
{
  /* stop distance calculation */
  int stop_idx = 0;
  const double stop_dist_lim = 50.0;
  double stop_dist = stop_dist_lim;
  if (vpu::searchZeroVelocityIdx(trajectory, stop_idx))
  {
    stop_dist = vpu::calcLengthOnWaypoints(trajectory, closest, stop_idx);
  }
  stop_dist = closest > stop_idx ? stop_dist : -stop_dist;
  std_msgs::Float32 dist_to_stopline;
  dist_to_stopline.data = std::max(-stop_dist_lim, std::min(stop_dist_lim, stop_dist));
  pub_dist_to_stopline_.publish(dist_to_stopline);
}

void MotionVelocityPlanner::calcInitialMotion(const double &base_speed, const autoware_planning_msgs::Trajectory &base_waypoints,
                                        const int base_closest, const std::vector<Motion> &prev_replanned_traj_motion,
                                        const int prev_replanned_traj_closest, MotionVelocityPlanner::Motion *initial_motion,
                                        int &init_type) const
{
  const double vehicle_speed = std::fabs(current_velocity_ptr_->twist.linear.x);

  if (prev_replanned_traj_motion.size() == 0 /* first time */)
  {
    initial_motion->vel = vehicle_speed;
    initial_motion->acc = 0.0;   // if possible, use actual vehicle acc & jerk value;
    initial_motion->jerk = 0.0;  // if possible, use actual vehicle acc & jerk value;
    init_type = 0;
    return;
  }

  const double desired_vel = prev_replanned_traj_motion.at(prev_replanned_traj_closest).vel;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);
  if (std::fabs(vel_error) > planning_param_.replan_vel_deviation /* when velocity tracking deviation is large */)
  {
    initial_motion->vel = vehicle_speed;  // use current vehicle speed
    initial_motion->acc = prev_replanned_traj_motion.at(prev_replanned_traj_closest).acc;
    initial_motion->jerk = 0.0;
    init_type = 1;
    DEBUG_WARN("[calcInitialMotion] : Large deviation error for speed control. Use current speed for initial value, "
               "desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
               desired_vel, vehicle_speed, vel_error, planning_param_.replan_vel_deviation);
    return;
  }

  /* if current vehicle velocity is low && base_desired speed is high, use engage_velocity for engage vehicle */
  if (vehicle_speed < 0.5 * planning_param_.engage_velocity && base_speed - vehicle_speed > planning_param_.engage_velocity)
  {
    int idx = 0;
    const bool ret = vpu::searchZeroVelocityIdx(base_waypoints, idx);
    const bool exist_stop_point = (idx >= base_closest) ? ret : false;

    const double stop_dist = std::sqrt(vpu::calcSquaredDist2d(base_waypoints.points.at(idx), base_waypoints.points.at(base_closest)));
    if (!exist_stop_point || stop_dist > planning_param_.stop_dist_to_prohibit_engage)
    {
      initial_motion->vel = planning_param_.engage_velocity;
      initial_motion->acc = planning_param_.engage_acceleration;
      initial_motion->jerk = 0.0;
      init_type = 2;
      DEBUG_INFO("[calcInitialMotion] : vehicle speed is low (%3.3f [m/s]), but desired speed is high (%3.3f [m/s]). "
                "Use engage speed (%3.3f [m/s]), stop_dist = %3.3f",
                vehicle_speed, base_speed, planning_param_.engage_velocity, stop_dist);
      return;
    }
    else
    {
      DEBUG_INFO("[calcInitialMotion] : engage condition, but stop point is close (dist = %3.3f). no engage.", stop_dist);
    }
  }

  /* normal update: use closest in prev_output */
  initial_motion->vel = prev_replanned_traj_motion.at(prev_replanned_traj_closest).vel;
  initial_motion->acc = prev_replanned_traj_motion.at(prev_replanned_traj_closest).acc;
  initial_motion->jerk = prev_replanned_traj_motion.at(prev_replanned_traj_closest).jerk;
  init_type = 3;
  DEBUG_INFO("[calcInitialMotion]: normal update initial_motion.vel = %f, acc = %f, jerk = %f, vehicle_speed = %f, "
             "base_speed = %f",
             initial_motion->vel, initial_motion->acc, initial_motion->jerk, vehicle_speed, base_speed);
  return;
}

void MotionVelocityPlanner::replanVelocity(const autoware_planning_msgs::Trajectory &input, const int input_closest,
                                     const std::vector<Motion> &prev_output_motion, const int prev_output_closest,
                                     autoware_planning_msgs::Trajectory &output, std::vector<Motion> &output_motion, double &stop_planning_jerk) const
{
  const double base_speed = std::fabs(input.points.at(input_closest).twist.linear.x);

  /* calculate initial motion for planning */
  Motion init_m;
  int init_type = 3;
  MotionVelocityPlanner::calcInitialMotion(base_speed, input, input_closest, prev_output_motion, prev_output_closest,
                                     /* out */ &init_m, /* out */ init_type);

  /* apply lateral acceleration filter */
  autoware_planning_msgs::Trajectory latacc_filtered_traj = input;
  const unsigned int idx_dist = 20;
  MotionVelocityPlanner::lateralAccelerationFilter(input, planning_param_.max_lateral_accel, idx_dist,
                                             /* out */ latacc_filtered_traj);

  /* apply forward move average filter for delay compensation */
  autoware_planning_msgs::Trajectory moveave_filtered_traj = latacc_filtered_traj;
  const unsigned int move_ave_num = 20;
  MotionVelocityPlanner::moveAverageFilter(latacc_filtered_traj, move_ave_num, /* out */ moveave_filtered_traj);

  /* apply forward-direction jerk filter */
  autoware_planning_msgs::Trajectory jerk_filtered_traj = moveave_filtered_traj;
  std::vector<Motion> jerk_filtered_motion;
  MotionVelocityPlanner::jerkVelocityFilter(init_m, moveave_filtered_traj, input_closest, prev_output_motion,
                                      prev_output_closest, init_type, 
                                      /* out */ jerk_filtered_traj, /* out */ jerk_filtered_motion);

  /* find stop point for stopVelocityFilter */
  int stop_idx_zero_vel = -1;
  bool stop_point_exists = vpu::searchZeroVelocityIdx(input, stop_idx_zero_vel);
  DEBUG_INFO("[replan] : base_speed = %f, stop_idx_zero_vel = %d, input_closest = %d, stop_point_exists = %d",
             base_speed, stop_idx_zero_vel, input_closest, (int)stop_point_exists);

  bool is_stop_ok = false;
  stop_planning_jerk = 0.0;
  if (emergency_stop_manager_ptr_->getEmergencyFlag() == true)
  {
    setZeroLaneAndMotions(input, output, output_motion);
    stop_planning_jerk = -77.7;
    DEBUG_WARN("[replan] : EMERGENCY DETECTED. Keep stopping for a while... ");
  }
  else if (stop_point_exists == false) /* no stop idx */
  {
    output = jerk_filtered_traj;
    output_motion = jerk_filtered_motion;
  }
  else if (stop_idx_zero_vel > input_closest) /* stop idx exist ahead of self position */
  {
    if (!stopVelocityFilterWithJerkRange(
            planning_param_.dec_jerk_urgent, planning_param_.dec_jerk_nominal, planning_param_.jerk_planning_span,
            planning_param_.stop_dist_mergin, stop_idx_zero_vel, jerk_filtered_traj, jerk_filtered_motion, input_closest,
            /* out */ stop_planning_jerk, /* out */ output, /* out */ is_stop_ok, /* out */ output_motion))
    {
      vpu::insertZeroVelocityAfterIdx(0, output);
      MotionVelocityPlanner::insertZeroMotionsAfterIdx(0, /* out */ output_motion);
      ROS_ERROR("error in stopVelocityFilter. publish zero velocity ");
      return;
    }

    if (is_stop_ok == false)  // cannot find stop velocity profile with any given constraint
    {
      setZeroLaneAndMotions(input, output, output_motion);
      if (std::fabs(current_velocity_ptr_->twist.linear.x) > planning_param_.emergency_flag_vel_thr_kmph / 3.6 /* 3.0 km/h */)
      {
        stop_planning_jerk = -88.8; // for debug
        double dist_tmp = vpu::calcLengthOnWaypoints(jerk_filtered_traj, stop_idx_zero_vel, input_closest);
        ROS_WARN_DELAYED_THROTTLE(5.0, "[replan] : could not plan under given jerk constraint. stop_idx = %d, self_idx = %d, dist = %f, v0 = %f, a0 "
                  "= %f",
                  stop_idx_zero_vel, input_closest, dist_tmp, init_m.vel, init_m.acc);
        emergency_stop_manager_ptr_->setEmergencyFlagTrue(); // EMERGENCY
      }
    }
  }
  else /* stop idx is behind self pose -> emergency */
  {
    setZeroLaneAndMotions(input, output, output_motion);
    if (std::fabs(current_velocity_ptr_->twist.linear.x) > planning_param_.emergency_flag_vel_thr_kmph / 3.6 /* 3.0 km/h */)
    {
      stop_planning_jerk = -99.9;
      emergency_stop_manager_ptr_->setEmergencyFlagTrue(); // EMERGENCY
    }
    else
    {
      /* stop idx is behind self pose, but current_velocity is low. Not emergency, use previous jerk for debug publish */
      stop_planning_jerk = prev_stop_planning_jerk_;
    }
    DEBUG_INFO("[replan] : over stop line. publish zero velocity. stop_idx_zero_vel = %d, input_closest = %d", stop_idx_zero_vel, input_closest);
  }

  /* set 0 velocity after stop index for safety */
  if (stop_point_exists == true)
  {
    vpu::insertZeroVelocityAfterIdx(stop_idx_zero_vel, /* out */ output);
    MotionVelocityPlanner::insertZeroMotionsAfterIdx(stop_idx_zero_vel, /* out */ output_motion);
  }

  /* for the endpoint of the trajectory */
  if (output.points.size() > 0)
    output.points.back().twist.linear.x = 0.0;
  if (output_motion.size() > 0)
  output_motion.back().vel = 0.0;

  /* check if it is emergency with planning jerk */
  publishIsEmergency(stop_planning_jerk);

  /* debug */
  publishPlanningJerk(stop_planning_jerk); // for debug
}

void MotionVelocityPlanner::publishIsEmergency(const double &jerk_value) const
{
  std_msgs::Bool is_emergency;
  if (std::fabs(jerk_value) > std::fabs(planning_param_.large_jerk_report))
  {
    is_emergency.data = enable_to_publish_emergency_;
  }
  else
  {
    is_emergency.data = false;
  }
  pub_is_emergency_.publish(is_emergency);
}

void MotionVelocityPlanner::setZeroLaneAndMotions(const autoware_planning_msgs::Trajectory &base_trajectory, autoware_planning_msgs::Trajectory &trajectory,
                                            std::vector<Motion> &motions) const
{
  trajectory = base_trajectory;
  motions.clear();
  Motion zero_motion(0.0, 0.0, 0.0);
  for (unsigned int i = 0; i < base_trajectory.points.size(); ++i)
  {
    trajectory.points.at(i).twist.linear.x = 0.0;
    motions.push_back(zero_motion);
  }
}

bool MotionVelocityPlanner::moveAverageFilter(const autoware_planning_msgs::Trajectory &input, const unsigned int move_ave_num,
                                        autoware_planning_msgs::Trajectory &moveave_filtered_traj) const
{
  if (enable_latacc_filter_ == false)
  {
    moveave_filtered_traj = input;
    return true;
  }

  moveave_filtered_traj = input;
  for (unsigned int i = 0; i < input.points.size(); ++i)
  {
    double vel_sum = 0;
    int count = 0;
    for (unsigned int j = 0; i + j < input.points.size() && j < move_ave_num; ++j)
    {
      vel_sum += input.points.at(i + j).twist.linear.x;
      ++count;
    }
    if (count == 0)
    {
      ROS_ERROR("[moveAverageFilter] something wrong.");
    }
    moveave_filtered_traj.points.at(i).twist.linear.x = vel_sum / (double)count;
  }
  return true;
}

bool MotionVelocityPlanner::lateralAccelerationFilter(const autoware_planning_msgs::Trajectory &input,
                                                const double &max_lateral_accel, const unsigned int idx_dist,
                                                autoware_planning_msgs::Trajectory &latacc_filtered_traj) const
{
  if (enable_latacc_filter_ == false)
  {
    latacc_filtered_traj = input;
    return true;
  }
  std::vector<double> k_arr;
  vpu::calcTrajectoryCurvatureFrom3Points(input, idx_dist, k_arr);

  latacc_filtered_traj = input;  // as initialize
  const double max_lateral_accel_abs = std::fabs(max_lateral_accel);

  for (unsigned int i = 0; i < input.points.size(); ++i)
  {
    const double curvature = std::max(std::fabs(k_arr.at(i)), 0.01 /* avoid 0 divide */);
    const double v_max = std::sqrt(max_lateral_accel_abs / curvature);
    if (latacc_filtered_traj.points.at(i).twist.linear.x > v_max)
    {
      latacc_filtered_traj.points.at(i).twist.linear.x = v_max;
    }
  }
  return true;
};

void MotionVelocityPlanner::insertZeroMotionsAfterIdx(const int &idx, std::vector<Motion> &motions) const
{
  for (int i = idx; i < (int)motions.size(); ++i)
  {
    motions.at(i).vel = 0;
    motions.at(i).acc = 0;
    motions.at(i).jerk = 0;
  }
  return;
}

/* update motion with constant jerk & acceleration constraint */
MotionVelocityPlanner::Motion MotionVelocityPlanner::updateMotionWithConstraint(const Motion &m_prev, const Motion &m_des_prev,
                                                                    const double &dt,
                                                                    const MotionVelocityPlannerParam &planning_param,
                                                                    int &debug) const
{
  debug = 0;

  if (std::fabs(dt) < 1.0E-8)
  {
    debug = -1;
    return m_prev;  // no update
  }

  Motion m_ret;
  const double curr_acc_max = m_prev.acc + planning_param.acc_jerk * dt;
  const double curr_acc_min = m_prev.acc + planning_param.dec_jerk_nominal * dt;

  if (planning_param.min_decel < m_des_prev.acc && m_des_prev.acc < planning_param.max_accel && curr_acc_min < m_des_prev.acc &&
      m_des_prev.acc < curr_acc_max)
  {
    // without any constraint
    m_ret.vel = m_prev.vel + m_des_prev.acc * dt;
    m_ret.acc = (m_ret.vel - m_prev.vel) / dt;
    m_ret.jerk = (m_ret.acc - m_prev.acc) / dt;
    debug = 0;
  }
  else
  {
    double jerk_des = m_des_prev.jerk;
    if (m_des_prev.jerk < planning_param.dec_jerk_nominal)
    {
      // with min jerk constraint
      jerk_des = planning_param.dec_jerk_nominal;
      debug += 1;
    }
    else if (planning_param.acc_jerk < m_des_prev.jerk)
    {
      // with max jerk constraint
      jerk_des = planning_param.acc_jerk;
      debug += 2;
    }

    const double acc_ideal = m_prev.acc + jerk_des * dt;
    double ratio = 1.0;
    double acc_lim = 0.0;
    if (acc_ideal < planning_param.min_decel)
    {
      // with acceleration constraint
      ratio = (planning_param.min_decel - m_prev.acc) / (acc_ideal - m_prev.acc);
      acc_lim = planning_param.min_decel;
      debug += 10;
    }
    else if (planning_param.max_accel < acc_ideal)
    {
      // with deceleration constraint
      ratio = (planning_param.max_accel - m_prev.acc) / (acc_ideal - m_prev.acc);
      acc_lim = planning_param.max_accel;
      debug += 20;
    }

    // integrate
    const double dt1 = dt * ratio;
    const double dt2 = dt - dt1;
    m_ret.vel = std::min(m_prev.vel + m_prev.acc * dt + 0.5 * jerk_des * dt1 * dt1 + acc_lim * dt2,
                         planning_param.max_velocity);
    m_ret.acc = std::min(std::max(m_prev.acc + jerk_des * dt1, planning_param.min_decel), planning_param.max_accel);
    m_ret.jerk = jerk_des;
  }

  return m_ret;
};

void MotionVelocityPlanner::jerkVelocityFilter(const Motion &initial_motion, const autoware_planning_msgs::Trajectory &input,
                                         const int input_closest, const std::vector<Motion> &prev_output_motion,
                                         const int prev_output_closest, const int &init_type, autoware_planning_msgs::Trajectory &output,
                                         std::vector<Motion> &output_motion) const
{
  DEBUG_INFO_ALL(" --- move plan start --- ");
  DEBUG_INFO("[jerkVelocityFilter]: constraint acc = %f, decel = %f, acc_jerk = %f, dec_jerk = %f",
             planning_param_.max_accel, planning_param_.min_decel, planning_param_.acc_jerk, planning_param_.dec_jerk_nominal);

  output = input;

  const double kp = planning_param_.velocity_feedback_gain;
  // const double kd = 0.0;

  output_motion.clear();
  DEBUG_INFO_ALL("input_closest = %d, output.size() = %d", input_closest, (int)output.points.size());

  /* For backward velocity: use prev_output info for visualize or smoothing */
  for (int i = 0; i < input_closest; ++i)
  {
#if 0
    int j = prev_output_closest - input_closest + i;
    if (0 <= j && j < (int)prev_output_motion.size())
    {
      Motion m_prev = prev_output_motion.at(j);
      output.points.at(i).twist.linear.x = m_prev.vel;
      output_motion.push_back(m_prev);
    }
    else
    {
      Motion zero(0., 0., 0.);
      output.points.at(i).twist.linear.x = zero.vel;
      output_motion.push_back(zero);
    }
#else
    output.points.at(i).twist.linear.x = initial_motion.vel;
    output_motion.push_back(initial_motion);
#endif
  }
  output.points.at(input_closest).twist.linear.x = initial_motion.vel;
  output_motion.push_back(initial_motion);

  /* update velocity for forward-direction with constraints */
  Motion m_prev = initial_motion;
  Motion m_des_prev;
  int idx = std::max(input_closest, 0);  // -1 at initial planning
  for (int i = idx + 1; i < (int)input.points.size(); ++i)
  {
    const double dt_prev = vpu::getDurationToNextIdx(input, m_prev.vel, i - 1);

    /* calculate previous desired motion */
    m_des_prev.vel = input.points.at(i - 1).twist.linear.x;
    const double acc_ref =
        std::min(std::max(vpu::getForwardAcc(input, i - 1), planning_param_.min_decel), planning_param_.max_accel);
    m_des_prev.acc = acc_ref - kp * (m_prev.vel - m_des_prev.vel);
    m_des_prev.jerk = (m_des_prev.acc - m_prev.acc) / dt_prev;

    /* calculate current motion from previous motion */
    int debug;
    Motion m_curr = updateMotionWithConstraint(m_prev, m_des_prev, dt_prev, planning_param_, debug);

    if (i == idx + 1 && init_type == 2)
    {
      m_curr = initial_motion; // copy just for once for oscillation
    }

    DEBUG_INFO_ALL("i = %d, ret.v = %f, ret.a = %f, ret.s = %f, prev.v = %f, prev.a = %f, prev.s = %f, des.v = %f, "
                   "des.acc = %f, des.s = %f, dt_prev = %f, kp = %f, debug = %d",
                   i, m_curr.vel, m_curr.acc, m_curr.jerk, m_prev.vel, m_prev.acc, m_prev.jerk, m_des_prev.vel,
                   m_des_prev.acc, m_des_prev.jerk, dt_prev, kp, debug);

    /* negative velocity is not allowed */
    if (m_curr.vel < 0.0)
    {
      DEBUG_WARN("[jerkVelocityFilter] negative celocity is not allowed. set to zero.");
      m_curr.vel = 0.0;
      m_curr.acc = 0.0;
      m_curr.jerk = 0.0;
      Motion zero(0.0, 0.0, 0.0);
    }

    /* insert replanned velocity */
    output_motion.push_back(m_curr);
    output.points.at(i).twist.linear.x = m_curr.vel;
    m_prev = m_curr;
  }

  DEBUG_INFO_ALL(" --- move plan end ---");
}

bool MotionVelocityPlanner::stopVelocityFilterWithJerkRange(const double &jerk_max, const double &jerk_min,
                                                      const double &jerk_span, const double &stop_mergin,
                                                      const int &input_stop_idx, const autoware_planning_msgs::Trajectory &input,
                                                      const std::vector<Motion> &input_motion, const int &input_closest,
                                                      double &planning_jerk, autoware_planning_msgs::Trajectory &output,
                                                      bool &is_stop_ok, std::vector<Motion> &output_motion) const
{
  static const double ep = 0.001;
  for (double stop_planning_jerk = jerk_min; stop_planning_jerk > jerk_max - ep; stop_planning_jerk -= jerk_span)
  {
    if (!stopVelocityFilterWithMergin(stop_mergin, input_stop_idx, input, input_motion,
                                      input_closest, stop_planning_jerk, /* out */ output,
                                      /* out */ is_stop_ok, /* out */ output_motion))
    {
      return false;
    }
    if (is_stop_ok == true)  // it can stop at stop point with given jerk constraint
    {
      DEBUG_INFO("[stopVelocityFilterWithJerkRange] plan successed, jerk = %f", stop_planning_jerk);
      planning_jerk = stop_planning_jerk;
      return true;
    }
  }

  planning_jerk = jerk_max;
  ROS_WARN_DELAYED_THROTTLE(5.0, "[stopVelocityFilterWithJerkRange] could not plan under jerk constraint");
  return true;
}


bool MotionVelocityPlanner::stopVelocityFilterWithMergin(const double &stop_mergin, const int &input_stop_idx,
                                                   const autoware_planning_msgs::Trajectory &input,
                                                   const std::vector<Motion> &input_motion, const int &input_closest,
                                                   const double &planning_jerk, autoware_planning_msgs::Trajectory &output,
                                                   bool &is_stop_ok, std::vector<Motion> &output_motion) const
{
  /* find stop point with mergin (input_stop_idx_with_mergin < input_stop_idx) */
  int input_stop_idx_with_mergin = input_stop_idx;
  double dist_tmp = 0.0;
  for (int i = input_stop_idx - 1; i >= 0; --i)
  {
    dist_tmp += std::sqrt(vpu::calcSquaredDist2d(input.points.at(i), input.points.at(i + 1)));
    if (dist_tmp < stop_mergin)
    {
      input_stop_idx_with_mergin = i;
    }
    else
    {
      break;
    }
  }

  /* Calculate stop_point with mergin */
  for (int i = input_stop_idx_with_mergin; i <= input_stop_idx; ++i)
  {
    if (!stopVelocityFilter(i, input, input_motion, input_closest, planning_jerk, output, is_stop_ok, output_motion))
    {
      return false; // something wrong
    }

    if (is_stop_ok)
    {
      double mergin = vpu::calcLengthOnWaypoints(input, i, input_stop_idx);
      DEBUG_INFO("[stopVelocityFilterWithMergin] plan successed, planning_jerk = %f, mergin = %f", planning_jerk, mergin);
      return true; // velocity plan successed under given constraint.
    }
  }
  ROS_WARN_DELAYED_THROTTLE(5.0, "[stopVelocityFilterWithMergin] could not plan under jerk constraint");

  is_stop_ok = false; // couldn't plan velocity under given constraint.
  return true; // no error, return true.
}

bool MotionVelocityPlanner::stopVelocityFilter(const int &input_stop_idx, const autoware_planning_msgs::Trajectory &input,
                                         const std::vector<Motion> &input_motion, const int &input_closest,
                                         const double &planning_jerk, autoware_planning_msgs::Trajectory &output, bool &is_stop_ok,
                                         std::vector<Motion> &output_motion) const
{
  output = input;                // as initialize
  output_motion = input_motion;  // as initialize
  is_stop_ok = false;

  /* No stop plan */
  if (input_stop_idx < 0)
  {
    DEBUG_INFO("[stopVelocityFilter]: return because no stop point");
    is_stop_ok = true;
    return true;  // do nothing
  }

  /* Emergency */
  if (input_stop_idx < input_closest || input_closest >= (int)output.points.size())
  {
    ROS_DEBUG("[stopVelocityFilter]: return, self_pose over stop_point");
    is_stop_ok = false;
    vpu::setZeroVelocity(output);
    return true;  // return value is for system error. return true with is_stop_ok = false for emergency.
  }

  /* Undesired jerk */
  if (planning_jerk > 0)
  {
    ROS_ERROR("[stopVelocityFilter]: return, planning_jerk is positive, must be negative.");
    return false;
  }

  /* Calculate arclength-vector of trajectory for stop distance calculation */
  double dist = 0.0;
  std::vector<double> dist_to_stop_arr;
  dist_to_stop_arr.push_back(dist);
  for (int i = input_stop_idx; i > 0; --i)
  {
    dist += std::sqrt(vpu::calcSquaredDist2d(input.points.at(i).pose, input.points.at(i - 1).pose));
    dist_to_stop_arr.insert(dist_to_stop_arr.begin(), dist);
  }

  /* Calculate stop distance under given jerk constraint.
   * If there is enough distance to stop under jerk constraint, find suitable start index for stop velocity calculation
   * in forward direction from self-pose index. Otherwise, it means the vehicle cannot stop under given constraint, so
   * set is_stop_ok = false, then return.
   */
  const double v_end_s = planning_param_.stopping_speed;   // this maybe >0 for lower controller to handle smooth stop
  const double v0_s = input_motion.at(input_closest).vel;  // initial velocity
  const double a0_s = input_motion.at(input_closest).acc;  // initial acceleration
  double t_neg_s;                                          // time for negative jerk
  double t_pos_s;                                          // time for positive jerk
  double brake_dist_s;                                     // stop distance under constraint

  if (!vpu::calcStopDistWithConstantJerk(v0_s, a0_s, planning_jerk, v_end_s, t_neg_s, t_pos_s, brake_dist_s))
  {
    ROS_WARN_DELAYED_THROTTLE(3.0, "[stopVelocityFilter]: cannot calculate stop_dist in calcStopDistWithConstantJerk() (planning_jerk = %f)", planning_jerk);
    is_stop_ok = false;
    return true;
  }
  DEBUG_INFO("[stopVelocityFilter]: check if distance is enough : jerk = %3.3f, v0 = %3.3f, a0 = %3.3f, needed_dist = %3.3f, dist_to_stopline = %3.3f, stop_idx = %d, closest = %d",
             planning_jerk, v0_s, a0_s, brake_dist_s, dist_to_stop_arr.at(input_closest), input_stop_idx, input_closest);

  /* Check if stop_distance under jerk constriant is enough */
  is_stop_ok = (0.0 <= brake_dist_s && brake_dist_s <= dist_to_stop_arr.at(input_closest));
  if (!is_stop_ok)
    return true;  // this should be true (is_stop_ok = false)

  /* Since there is enough distance to stop, find start point from stop point toward vehicle current point */
  bool start_point_found = false;
  std::vector<double> a_arr, s_arr;
  int plan_start_index = -1;

  int debug_type = 0;
  for (int i = input_stop_idx - 1; i >= input_closest; --i)
  {
    double v0, a0;
    if (input_motion.size() == 0)
    {
      debug_type = 777;
      v0 = vpu::getVx(input, i);
      a0 = vpu::getForwardAcc(input, i);
    }
    else
    {
      debug_type = 1;
      v0 = input_motion.at(i).vel;
      a0 = input_motion.at(i).acc;
    }

    double t_neg, t_pos;
    double brake_dist;
    if(vpu::calcStopDistWithConstantJerk(v0, a0, planning_jerk, v_end_s, t_neg, t_pos, brake_dist))
    {
      // printf("[stopVelocityFilter]: searching start idx for stop planning : i = %d, v = %3.3f, a = %3.3f, brake_dist = %3.3f, \n", i, v0, a0, brake_dist);
      start_point_found = (0.0 <= brake_dist && brake_dist < dist_to_stop_arr.at(i));
      if (start_point_found)
      {
        plan_start_index = i;
        if (!vpu::calcStopVelocityWithConstantJerk(v0, a0, planning_jerk, t_neg, t_pos, i, output, a_arr, s_arr))
        {
          ROS_ERROR("cannot find stop point!!!! Something wrong.bbb");
          return false;
        }
        DEBUG_INFO("[stopVelocityFilter]: stop planning start point found : jerk = %3.3f, start_idx = %d"
                  " (v = %3.3f, a = %3.3f), brake_dist = %3.3f, stop_idx = %d (debug_type:%d)",
                  planning_jerk, plan_start_index, v0, a0, brake_dist, input_stop_idx, debug_type);
        break;
      }
    }    
  }

  if (plan_start_index == -1)
  {
    ROS_ERROR("stop_dist is enough, but could not find stop point. Something wrong. (planning_jerk = %f)", planning_jerk);
    return false;
  }



  /* copy stop plan result to outputs */
  for (int i = plan_start_index; i < (int)output_motion.size(); ++i)
  {
    output_motion.at(i).vel = output.points.at(i).twist.linear.x;
  }
  for (int i = 0; i < (int)a_arr.size(); ++i)
  {
    output_motion.at(plan_start_index + i).acc = a_arr.at(i);
    output_motion.at(plan_start_index + i).jerk = s_arr.at(i);
  }
  for (int i = input_stop_idx; i < (int)output_motion.size(); ++i)
  {
    output_motion.at(i).acc = 0.0;
    output_motion.at(i).jerk = 0.0;
  }
  return true;
}

void MotionVelocityPlanner::calculateMotionsFromWaypoints(const autoware_planning_msgs::Trajectory &trajectory, std::vector<Motion> motions) const
{
  std::vector<double> s;
  vpu::calcWaypointsArclength(trajectory, s);
  Motion m;
  for (int i = 0; i < (int)trajectory.points.size(); ++i)
  {
    m.vel = trajectory.points.at(i).twist.linear.x;
    motions.push_back(m);
  }
  for (int i = 1; i < (int)trajectory.points.size() - 1; ++i)
  {
    const double v0 = trajectory.points.at(i - 1).twist.linear.x;
    const double v1 = trajectory.points.at(i).twist.linear.x;
    const double v2 = trajectory.points.at(i + 1).twist.linear.x;
    const double ds = std::max(s.at(i) - s.at(i - 1), 0.001);
    motions.at(i).acc = (v1 * v1 - v0 * v0) / 2.0 / ds;
    const double c0 = s.at(i - 1) * s.at(i);
    const double c1 = s.at(i - 1) * s.at(i + 1);
    const double c2 = s.at(i) * s.at(i + 1);
    const double tmp1 = (c0 - c1 - c2 + s.at(i + 1) * s.at(i + 1));
    const double tmp2 = (c0 + c1 - c2 - s.at(i - 1) * s.at(i - 1));
    const double tmp3 = (c0 - c1 + c2 - s.at(i) * s.at(i + 1));
    motions.at(i).jerk = 2.0 * v2 / tmp1 - 2.0 * v0 / tmp2 - 2.0 * v1 / tmp3;
  }
}

void MotionVelocityPlanner::preventMoveToVeryCloseStopLine(const int closest, autoware_planning_msgs::Trajectory &trajectory) const
{
  if (std::fabs(current_velocity_ptr_->twist.linear.x) < 0.01)
  {
    int stop_idx = 0;
    bool stop_point_exist = vpu::searchZeroVelocityIdx(trajectory, stop_idx);
    if (stop_point_exist && stop_idx >= closest /* desired stop line is ahead of ego-vehicle */)
    {
      double stop_dist = std::sqrt(vpu::calcSquaredDist2d(trajectory.points.at(stop_idx), trajectory.points.at(closest)));
      if (stop_dist < planning_param_.stop_dist_to_prohibit_engage)
      {
        vpu::setZeroVelocity(trajectory);
        DEBUG_INFO("[preventMoveToVeryCloseStopLine] set zero vel curr_vel = %3.3f, stop_dist = %3.3f < thr = %3.3f",
                   current_velocity_ptr_->twist.linear.x, stop_dist, planning_param_.stop_dist_to_prohibit_engage);
      }
    }
  }
}

void MotionVelocityPlanner::publishClosestVelocity(const double &vel) const
{
  std_msgs::Float32 closest_velocity;
  closest_velocity.data = vel;
  debug_closest_velocity_.publish(closest_velocity);
};

void MotionVelocityPlanner::publishPlanningJerk(const double &jerk) const
{
  std_msgs::Float32 planning_jerk;
  planning_jerk.data = jerk;
  pub_debug_planning_jerk_.publish(planning_jerk);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_velocity_planner");
  MotionVelocityPlanner obj;

  ros::spin();

  return 0;
};

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

#include <motion_velocity_planner/motion_velocity_planner_osqp.hpp>
#include <chrono>

// clang-format off
#define DEBUG_INFO(...) { if (show_debug_info_) {ROS_INFO(__VA_ARGS__); } }
#define DEBUG_WARN(...) { if (show_debug_info_) {ROS_WARN(__VA_ARGS__); } }
#define DEBUG_INFO_ALL(...) { if (show_debug_info_all_) {ROS_INFO(__VA_ARGS__); } }

// clang-format on
MotionVelocityPlanner::MotionVelocityPlanner() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.param("max_velocity", planning_param_.max_velocity, double(20.0));          // 72.0 kmph
  pnh_.param("max_accel", planning_param_.max_accel, double(2.0));                 // 0.11G
  pnh_.param("min_decel", planning_param_.min_decel, double(-3.0));                // -0.2G
  pnh_.param("max_lateral_accel", planning_param_.max_lateral_accel, double(0.2)); //
  pnh_.param("replan_vel_deviation", planning_param_.replan_vel_deviation, double(3.0));
  pnh_.param("engage_velocity", planning_param_.engage_velocity, double(0.3));
  pnh_.param("engage_acceleration", planning_param_.engage_acceleration, double(0.1));
  pnh_.param("extract_ahead_dist", planning_param_.extract_ahead_dist, double(200.0));
  pnh_.param("extract_behind_dist", planning_param_.extract_behind_dist, double(3.0));
  pnh_.param("max_trajectory_length", planning_param_.max_trajectory_length, double(200.0));
  pnh_.param("min_trajectory_length", planning_param_.min_trajectory_length, double(30.0));
  pnh_.param("resample_total_time", planning_param_.resample_total_time, double(10.0));
  pnh_.param("resample_time_interval", planning_param_.resample_dt, double(0.1));
  pnh_.param("min_trajectory_interval_distance", planning_param_.min_trajectory_interval_distance, double(0.1));
  pnh_.param("stop_dist_not_to_drive_vehicle", planning_param_.stop_dist_not_to_drive_vehicle, double(1.5));
  pnh_.param("stop_dist_mergin", planning_param_.stop_dist_mergin, double(0.55));

  pnh_.param("show_debug_info", show_debug_info_, bool(true));
  pnh_.param("show_debug_info_all", show_debug_info_all_, bool(false));
  pnh_.param("show_figure", show_figure_, bool(false));
  pnh_.param("publish_debug_trajs", publish_debug_trajs_, bool(false));

  pnh_.param("pseudo_jerk_weight", qp_param_.pseudo_jerk_weight, double(1000.0));
  pnh_.param("over_v_weight", qp_param_.over_v_weight, double(100000.0));
  pnh_.param("over_a_weight", qp_param_.over_a_weight, double(1000.0));

  pub_trajectory_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/trajectory", 1);
  pub_dist_to_stopline_ = pnh_.advertise<std_msgs::Float32>("distance_to_stopline", 1);
  sub_current_trajectory_ = pnh_.subscribe("input/trajectory", 1, &MotionVelocityPlanner::callbackCurrentTrajectory, this);
  sub_current_velocity_ = pnh_.subscribe("/vehicle/status/twist", 1, &MotionVelocityPlanner::callbackCurrentVelocity, this);
  sub_external_velocity_limit_ = pnh_.subscribe("external_velocity_limit_mps", 1, &MotionVelocityPlanner::callbackExternalVelocityLimit, this);

  /* dynamic reconfigure */
  dynamic_reconfigure::Server<motion_velocity_planner::MotionVelocityPlannerConfig>::CallbackType dyncon_f =
      boost::bind(&MotionVelocityPlanner::dynamicRecofCallback, this, _1, _2);
  dyncon_server_.setCallback(dyncon_f);

  /* debug */
  debug_closest_velocity_ = pnh_.advertise<std_msgs::Float32>("closest_velocity", 1);
  pub_trajectory_raw_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_raw", 1);
  pub_trajectory_vel_lim_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_external_velocity_limitted", 1);
  pub_trajectory_latcc_filtered_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_lateral_acc_filtered", 1);
  pub_trajectory_resampled_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("debug/trajectory_time_resampled", 1);

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
  run();
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
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
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

void MotionVelocityPlanner::run()
{
  auto t_start = std::chrono::system_clock::now();

  DEBUG_INFO("============================== run() start ==============================");

  updateCurrentPose();

  /* guard */
  if (!current_pose_ptr_ || !current_velocity_ptr_ || !base_traj_raw_ptr_ || base_traj_raw_ptr_->points.size() == 0)
  {
    DEBUG_INFO("wait topics : current_pose = %d, current_vel = %d, base_traj = %d, traj.size = %lu", (bool)current_pose_ptr_,
               (bool)current_velocity_ptr_, (bool)base_traj_raw_ptr_, base_traj_raw_ptr_->points.size());
    return;
  }

  /* calculate trajectory velocity */
  autoware_planning_msgs::Trajectory output = calcTrajectoryVelocity(*base_traj_raw_ptr_);

  /* publish message */
  output.header = base_traj_raw_ptr_->header;
  publishTrajectory(output);

  prev_output_ = output;

  auto t_end = std::chrono::system_clock::now();
  double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() * 1.0e-6;
  DEBUG_INFO("run: calculation time = %f [ms]", elapsed_ms);
  DEBUG_INFO("============================== run() end ==============================\n\n");
}

autoware_planning_msgs::Trajectory MotionVelocityPlanner::calcTrajectoryVelocity(const autoware_planning_msgs::Trajectory &base_traj)
{

  /* (1) Find the nearest point to base_waypoints. This is not severe so, could read closest topic by callback. */
  int base_raw_closest = vpu::calcClosestWaypoint(base_traj, current_pose_ptr_->pose);
  if (base_raw_closest < 0)
  {
    ROS_WARN("[velocity planner] cannot find closest waypoint for base raw trajectory");
    return prev_output_;
  }

  /* (2) extruct the route surrounding the self-position from base_waypoints */
  autoware_planning_msgs::Trajectory base_traj_extracted;
  if (!vpu::extractPathAroundIndex(base_traj, base_raw_closest, planning_param_.extract_ahead_dist,
                                   planning_param_.extract_behind_dist, /* out */ base_traj_extracted))
  {
    ROS_WARN("extractPathAroundIndex failed. base_traj.size() = %lu, "
             "base_raw_closest = %d, extract_ahead_dist = %f, extract_behind_dist = %f",
             base_traj.points.size(), base_raw_closest, planning_param_.extract_ahead_dist,
             planning_param_.extract_behind_dist);
    return prev_output_;
  }
  DEBUG_INFO("[extractPathAroundIndex] : base_raw.size() = %lu, base_closest = %d, base_raw_extracted.size() = %lu",
             base_traj.points.size(), base_raw_closest, base_traj_extracted.points.size());
  if (publish_debug_trajs_)
  {
    pub_trajectory_raw_.publish(base_traj_extracted);
  }

  /* (3) Apply external velocity limit */
  if (external_velocity_limit_ptr_ != nullptr)
  {
    DEBUG_INFO("[motion_velocity_planner] : apply external velocity lim : %3.3f, base_closest_vel = %3.3f",
               external_velocity_limit_ptr_->data, base_traj.points.at(base_raw_closest).twist.linear.x);
    vpu::maximumVelocityFilter(external_velocity_limit_ptr_->data, base_traj_extracted);
  }
  if (publish_debug_trajs_)
  {
    pub_trajectory_vel_lim_.publish(base_traj_extracted);
  }

  /* apply lateral acceleration filter */
  const double curvature_calc_dist = 3.0;
  const unsigned int idx_dist = std::max((int)(curvature_calc_dist / std::max(0.1, 0.001)), 1);
  autoware_planning_msgs::Trajectory traj_latacc_filtered;
  lateralAccelerationFilter(base_traj_extracted, planning_param_.max_lateral_accel, idx_dist, /* out */ traj_latacc_filtered);
  if (publish_debug_trajs_)
  {
    pub_trajectory_latcc_filtered_.publish(traj_latacc_filtered);
  }

  /* (4) Resample extructed-waypoints with interpolation */
  autoware_planning_msgs::Trajectory traj_resampled;
  std::vector<double> interval_dist_arr;
  if (!resampleTrajectory(traj_latacc_filtered, /* out */ traj_resampled, interval_dist_arr))
  {
    return prev_output_;
  }
  int traj_resampled_closest = vpu::calcClosestWaypoint(traj_resampled, current_pose_ptr_->pose);
  if (publish_debug_trajs_)
  {
    pub_trajectory_resampled_.publish(traj_resampled);
  }

  /* publish stop distance */
  publishStopDistance(traj_resampled, traj_resampled_closest);

  /* Change base velocity to zero when current_velocity == 0 & stop_dist is close */
  preventMoveToVeryCloseStopLine(traj_resampled_closest, planning_param_.stop_dist_not_to_drive_vehicle, traj_resampled);

  /* for negative velocity */
  const bool negative_velocity_flag =
      traj_resampled.points.at(traj_resampled_closest).twist.linear.x < 0.0 ? true : false;
  if (negative_velocity_flag == true)
  {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, /* out */ traj_resampled);
  }

  /* (5) Calculate the nearest point on the previously planned traj (used to get initial planning speed) */
  int prev_output_closest = -1;
  if (prev_output_.points.size() != 0 /* if this is not initial planning */)
  {
    prev_output_closest = vpu::calcClosestWaypoint(prev_output_, current_pose_ptr_->pose);
  }
  DEBUG_INFO("[calcClosestWaypoint] for base_resampled : base_resampled.size() = %d, prev_planned_closest_ = %d",
             (int)traj_resampled.points.size(), prev_output_closest);

  /* (6) Replan velocity */
  autoware_planning_msgs::Trajectory output;
  optimizeVelocity(traj_resampled, traj_resampled_closest, prev_output_, prev_output_closest,
                   interval_dist_arr, /* out */ output);
  DEBUG_INFO("[optimizeVelocity] : current_replanned.size() = %d", (int)output.points.size());

  /* (7) max velocity filter for safety */
  vpu::maximumVelocityFilter(planning_param_.max_velocity, output);

  /* insert behind velocity for consistency */
  insertBehindVelocity(prev_output_closest, prev_output_, traj_resampled_closest, output);

  /* for negative velocity */
  if (negative_velocity_flag == true)
  {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, output);
  }

  /* for debug */
  MotionVelocityPlanner::publishClosestVelocity(output.points.at(traj_resampled_closest).twist.linear.x);

  return output;
};

void MotionVelocityPlanner::insertBehindVelocity(const int prev_output_closest, const autoware_planning_msgs::Trajectory &prev_output,
                                                 const int output_closest, autoware_planning_msgs::Trajectory &output)
{
  int j = std::max(prev_output_closest - 1, 0);
  for (int i = output_closest - 1; i >= 0; --i)
  {
    if (planning_type_ == 3)
    {
      output.points.at(i).twist.linear.x = prev_output.points.at(j).twist.linear.x;
    }
    else
    {
      output.points.at(i).twist.linear.x = output.points.at(output_closest).twist.linear.x;
    }
    
    j = std::max(j - 1, 0);
  }
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

bool MotionVelocityPlanner::resampleTrajectory(const autoware_planning_msgs::Trajectory &input,
                                               autoware_planning_msgs::Trajectory &output,
                                               std::vector<double> &interval_dist_arr) const
{
  std::vector<double> in_arclength;
  vpu::calcWaypointsArclength(input, in_arclength);
  const double Nt = planning_param_.resample_total_time / std::max(planning_param_.resample_dt, 0.001);
  const double ds_nominal = std::max(current_velocity_ptr_->twist.linear.x * planning_param_.resample_dt,
                                     planning_param_.min_trajectory_interval_distance);
  const double Ns = planning_param_.min_trajectory_length / std::max(ds_nominal, 0.001);
  const double N = std::max(Nt, Ns);
  std::vector<double> out_arclength;
  double dist_i = 0.0;
  out_arclength.push_back(dist_i);
  bool is_end_point = false;
  for (int i = 1; i <= N; ++i)
  {
    double ds = ds_nominal;
    if (i > Ns)
    {
      ds = std::max(1.0, ds_nominal); // if the planning time is not enough to see the desired distance, change the interval distance to see far.
    }
    dist_i += ds;
    if (dist_i > planning_param_.max_trajectory_length)
    {
      break; // distance is over max.
    }
    if (dist_i >= in_arclength.back())
    {
      is_end_point = true; // distance is over input endpoint.
      break;
    }
    out_arclength.push_back(dist_i);
    interval_dist_arr.push_back(ds);
  }
  if (!vpu::linearInterpTrajectory(in_arclength, input, out_arclength, output))
  {
    ROS_WARN("[motion_velocity_planner]: fail trajectory interpolation. size : in_arclength = %lu, input = %lu, out_arclength = %lu, output = %lu",
             in_arclength.size(), input.points.size(), out_arclength.size(), output.points.size());
    return false;
  }

  // add end point directly to consider the endpoint velocity.
  if (is_end_point)
  {
    const double ds_end = vpu::calcDist2d(output.points.back().pose, input.points.back().pose);
    output.points.push_back(input.points.back());
    interval_dist_arr.push_back(ds_end);
  }
  return true;
}

void MotionVelocityPlanner::calcInitialMotion(const double &base_speed, const autoware_planning_msgs::Trajectory &base_waypoints,
                                              const int base_closest, const autoware_planning_msgs::Trajectory &prev_output,
                                              const int prev_replanned_traj_closest, double &initial_vel, double &initial_acc)
{
  const double vehicle_speed = std::fabs(current_velocity_ptr_->twist.linear.x);

  if (prev_output.points.size() == 0 /* first time */)
  {
    initial_vel = vehicle_speed;
    initial_acc = 0.0; // if possible, use actual vehicle acc & jerk value;
    planning_type_ = 0;
    return;
  }

  const double desired_vel = prev_output.points.at(prev_replanned_traj_closest).twist.linear.x;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);
  if (std::fabs(vel_error) > planning_param_.replan_vel_deviation /* when velocity tracking deviation is large */)
  {
    initial_vel = vehicle_speed; // use current vehicle speed
    initial_acc = prev_output.points.at(prev_replanned_traj_closest).accel.linear.x;
    DEBUG_WARN("[calcInitialMotion] : Large deviation error for speed control. Use current speed for initial value, "
               "desired_vel = %f, vehicle_speed = %f, vel_error = %f, error_thr = %f",
               desired_vel, vehicle_speed, vel_error, planning_param_.replan_vel_deviation);
    planning_type_ = 1;
    return;
  }

  /* if current vehicle velocity is low && base_desired speed is high, use engage_velocity for engage vehicle */
  if (vehicle_speed < 0.5 * planning_param_.engage_velocity && base_speed - vehicle_speed > planning_param_.engage_velocity)
  {
    int idx = 0;
    const bool ret = vpu::searchZeroVelocityIdx(base_waypoints, idx);
    const bool exist_stop_point = (idx >= base_closest) ? ret : false;

    const double stop_dist = vpu::calcDist2d(base_waypoints.points.at(idx), base_waypoints.points.at(base_closest));
    if (!exist_stop_point || stop_dist > planning_param_.stop_dist_not_to_drive_vehicle)
    {
      initial_vel = planning_param_.engage_velocity;
      initial_acc = planning_param_.engage_acceleration;
      DEBUG_INFO("[calcInitialMotion] : vehicle speed is low (%3.3f [m/s]), but desired speed is high (%3.3f [m/s]). "
                 "Use engage speed (%3.3f [m/s]), stop_dist = %3.3f",
                 vehicle_speed, base_speed, planning_param_.engage_velocity, stop_dist);
      planning_type_ = 2;
      return;
    }
    else
    {
      DEBUG_INFO("[calcInitialMotion] : engage condition, but stop point is close (dist = %3.3f). no engage.", stop_dist);
    }
  }

  /* normal update: use closest in prev_output */
  initial_vel = prev_output.points.at(prev_replanned_traj_closest).twist.linear.x;
  initial_acc = prev_output.points.at(prev_replanned_traj_closest).accel.linear.x;
  DEBUG_INFO("[calcInitialMotion]: normal update initial_motion.vel = %f, acc = %f, vehicle_speed = %f, "
             "base_speed = %f",
             initial_vel, initial_acc, vehicle_speed, base_speed);
  planning_type_ = 3;
  return;
}

void MotionVelocityPlanner::solveOptimization(const double initial_vel, const double initial_acc, const autoware_planning_msgs::Trajectory &input,
                                              const int closest, const std::vector<double> &interval_dist_arr,
                                              autoware_planning_msgs::Trajectory &output)
{
  auto ts = std::chrono::system_clock::now();

  output = input;

  if ((int)input.points.size() < closest)
  {
    ROS_WARN("[MotionVelocityPlanner::solveOptimization] invalid closest.");
    return;
  }

  if (std::fabs(input.points.at(closest).twist.linear.x) < 0.1)
  {
    DEBUG_INFO("[MotionVelocityPlanner::solveOptimization] closest vmax < 0.1, keep stopping. return.");
    return;
  }

  const unsigned int N = input.points.size() - closest;

  if (N < 2)
  {
    return;
  }

  std::vector<double> vmax(N, 0.0);
  for (unsigned int i = 0; i < N; ++i)
  {
    vmax.at(i) = input.points.at(i + closest).twist.linear.x;
  }

  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3 * N + 1, 4 * N); // the matrix size depends on constraint numbers.

  std::vector<double> lower_bound(3 * N + 1, 0.0);
  std::vector<double> upper_bound(3 * N + 1, 0.0);

  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(4 * N, 4 * N);
  std::vector<double> q(4 * N, 0.0);

  /*
   * x = [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigme1, ..., sigmaN] in R^{4N}
   * b: velocity^2
   * a: acceleration
   * delta: 0 < bi < vmax^2 + delta
   * sigma: amin < ai - sigma < amax
   */

  const double amax = planning_param_.max_accel;
  const double amin = planning_param_.min_decel;
  const double smooth_weight = qp_param_.pseudo_jerk_weight;
  const double over_v_weight = qp_param_.over_v_weight;
  const double over_a_weight = qp_param_.over_a_weight;

  /* design objective function */
  for (unsigned int i = 0; i < N; ++i) // bi
  {
    // The lower the speed, the higher the demand of speed accuracy
    const double velocity_weight = 10.0 / (std::max(std::min(vmax[i], /*max=*/10.0), /*min=*/2.0));

    // w_v * |vmax^2 - b| -> minimize (-w_v * bi)
    q[i] = -velocity_weight;
  }

  for (unsigned int i = N; i < 2 * N - 1; ++i) // pseudo jerk: d(ai)/ds
  {
    unsigned int j = i - N;
    const double w_x_dsinv = smooth_weight * (1.0 / std::max(interval_dist_arr.at(j), 0.0001));
    P(i, i) += w_x_dsinv;
    P(i, i + 1) -= w_x_dsinv;
    P(i + 1, i) -= w_x_dsinv;
    P(i + 1, i + 1) += w_x_dsinv;
  }

  for (unsigned int i = 2 * N; i < 3 * N; ++i) // over velocity cost
  {
    P(i, i) += over_v_weight;
  }

  for (unsigned int i = 3 * N; i < 4 * N; ++i) // over acceleration cost
  {
    P(i, i) += over_a_weight;
  }

  /* design constraint matrix */
  // 0 < b - delta < vmax^2
  // NOTE: The delta allows b to be negative. This is actully invalid because the definition is b=v^2.
  // But mathematically, the strict b>0 constraint may make the problem infeasible, such as the case of
  // v=0 & a<0. To avoid the infesibility, we allow b<0. The negative b is dealt as b=0 when it is
  // converted to v with sqrt. If the weight of delta^2 is large (the value of delta is very small),
  // b is almost 0, and is not a big problem.
  for (unsigned int i = 0; i < N; ++i)
  {
    const int j = 2 * N + i;
    A(i, i) = 1.0;  // b_i
    A(i, j) = -1.0; // -delta_i
    upper_bound[i] = vmax[i] * vmax[i];
    lower_bound[i] = 0.0;
  }

  // amin < a - sigma < amax
  for (unsigned int i = N; i < 2 * N; ++i)
  {
    const int j = 2 * N + i;
    A(i, i) = 1.0;  // a_i
    A(i, j) = -1.0; // -sigma_i
    upper_bound[i] = amax;
    lower_bound[i] = amin;
  }

  // b' = 2a
  for (unsigned int i = 2 * N; i < 3 * N - 1; ++i)
  {
    const unsigned int j = i - 2 * N;
    const double dsinv = 1.0 / std::max(interval_dist_arr.at(j), 0.0001);
    A(i, j) = -dsinv;
    A(i, j + 1) = dsinv;
    A(i, j + N) = -2.0;
    upper_bound[i] = 0.0;
    lower_bound[i] = 0.0;
  }

  // initial condition
  const double v0 = initial_vel;
  {
    const unsigned int i = 3 * N - 1;
    A(i, 0) = 1.0; // b0
    upper_bound[i] = v0 * v0;
    lower_bound[i] = v0 * v0;

    A(i + 1, N) = 1.0; // a0
    upper_bound[i + 1] = initial_acc;
    lower_bound[i + 1] = initial_acc;
  }

  auto tf1 = std::chrono::system_clock::now();
  double elapsed_ms1 = std::chrono::duration_cast<std::chrono::nanoseconds>(tf1 - ts).count() * 1.0e-6;

  /* execute optimization */
  auto ts2 = std::chrono::system_clock::now();
  std::tuple<std::vector<double>, std::vector<double>, int> result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);

  // [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigme1, ..., sigmaN]
  const std::vector<double> optval = std::get<0>(result);

  /* get velocity & acceleration */
  for (int i = 0; i < closest; ++i)
  {
    double v = optval.at(0);
    output.points.at(i).twist.linear.x = std::sqrt(std::max(v, 0.0));
    output.points.at(i).accel.linear.x = optval.at(N);
  }
  for (unsigned int i = 0; i < N; ++i)
  {
    double v = optval.at(i);
    output.points.at(i + closest).twist.linear.x = std::sqrt(std::max(v, 0.0));
    output.points.at(i + closest).accel.linear.x = optval.at(i + N);
  }
  for (unsigned int i = N + closest; i < output.points.size(); ++i)
  {
    output.points.at(i).twist.linear.x = 0.0;
    output.points.at(i).accel.linear.x = 0.0;
  }

  DEBUG_INFO_ALL("[after optimize] idx, vel, acc, over_vel, over_acc ");
  for (unsigned int i = 0; i < N; ++i)
  {
    DEBUG_INFO_ALL("i = %d, v: %f, vmax: %f a: %f, b: %f, delta: %f, sigma: %f\n", i, std::sqrt(optval.at(i)), vmax[i], optval.at(i + N), optval.at(i), optval.at(i + 2 * N), optval.at(i + 3 * N));
  }

  auto tf2 = std::chrono::system_clock::now();
  double elapsed_ms2 = std::chrono::duration_cast<std::chrono::nanoseconds>(tf2 - ts2).count() * 1.0e-6;
  DEBUG_INFO("[optimization] initialization time = %f [ms], optimization time = %f [ms]", elapsed_ms1, elapsed_ms2);
};

void MotionVelocityPlanner::optimizeVelocity(const autoware_planning_msgs::Trajectory &input, const int input_closest,
                                             const autoware_planning_msgs::Trajectory &prev_output, const int prev_output_closest,
                                             const std::vector<double> &interval_dist_arr, autoware_planning_msgs::Trajectory &output)
{
  const double base_speed = std::fabs(input.points.at(input_closest).twist.linear.x);

  /* calculate initial motion for planning */
  double initial_vel = 0.0;
  double initial_acc = 0.0;
  calcInitialMotion(base_speed, input, input_closest, prev_output, prev_output_closest,
                    /* out */ initial_vel, initial_acc);

  autoware_planning_msgs::Trajectory optimized_traj;
  solveOptimization(initial_vel, initial_acc, input, input_closest, interval_dist_arr, /* out */ optimized_traj);

  /* find stop point for stopVelocityFilter */
  int stop_idx_zero_vel = -1;
  bool stop_point_exists = vpu::searchZeroVelocityIdx(input, stop_idx_zero_vel);
  DEBUG_INFO("[replan] : base_speed = %f, stop_idx_zero_vel = %d, input_closest = %d, stop_point_exists = %d",
             base_speed, stop_idx_zero_vel, input_closest, (int)stop_point_exists);

  /* for the endpoint of the trajectory */
  if (optimized_traj.points.size() > 0)
  {
    optimized_traj.points.back().twist.linear.x = 0.0;
  }

  /* set output trajectory */
  output = optimized_traj;
}

bool MotionVelocityPlanner::lateralAccelerationFilter(const autoware_planning_msgs::Trajectory &input,
                                                      const double &max_lateral_accel, const unsigned int curvature_calc_idx_dist,
                                                      autoware_planning_msgs::Trajectory &output) const
{
  output = input; // initialize

  std::vector<double> curvature_v;
  vpu::calcTrajectoryCurvatureFrom3Points(input, curvature_calc_idx_dist, curvature_v);

  const double max_lateral_accel_abs = std::fabs(max_lateral_accel);

  for (unsigned int i = 0; i < input.points.size(); ++i)
  {
    const double curvature = std::max(std::fabs(curvature_v.at(i)), 0.0001 /* avoid 0 divide */);
    const double v_curvature_max = std::max(std::sqrt(max_lateral_accel_abs / curvature), planning_param_.engage_velocity);
    if (output.points.at(i).twist.linear.x > v_curvature_max)
    {
      output.points.at(i).twist.linear.x = v_curvature_max;
    }
  }
  return true;
};

void MotionVelocityPlanner::preventMoveToVeryCloseStopLine(const int closest, const double move_dist_min, autoware_planning_msgs::Trajectory &trajectory) const
{
  if (std::fabs(current_velocity_ptr_->twist.linear.x) < 0.1)
  {
    int stop_idx = 0;
    bool stop_point_exist = vpu::searchZeroVelocityIdx(trajectory, stop_idx);
    double dist_to_stopline = -1.0;
    if (stop_point_exist && stop_idx >= closest /* desired stop line is ahead of ego-vehicle */)
    {
      dist_to_stopline = vpu::calcDist2d(trajectory.points.at(stop_idx), trajectory.points.at(closest));
      if (dist_to_stopline < move_dist_min)
      {
        vpu::setZeroVelocity(trajectory);
        DEBUG_INFO("[preventMoveToVeryCloseStopLine] vehicle velocity is low, and stop point is very close. keep stopping."
                   " curr_vel = %3.3f, dist_to_stopline = %3.3f < move_dist_min = %3.3f, stop_idx = %d, closest = %d",
                   current_velocity_ptr_->twist.linear.x, dist_to_stopline, move_dist_min, stop_idx, closest);
        return;
      }
    }
    DEBUG_INFO("[preventMoveToVeryCloseStopLine] vehicle velocity is low, and stop point is far away. move."
               " curr_vel = %3.3f, dist_to_stopline = %3.3f < move_dist_min = %3.3f, stop_idx = %d, closest = %d",
               current_velocity_ptr_->twist.linear.x, dist_to_stopline, move_dist_min, stop_idx, closest);
  }
}

void MotionVelocityPlanner::publishClosestVelocity(const double &vel) const
{
  std_msgs::Float32 closest_velocity;
  closest_velocity.data = vel;
  debug_closest_velocity_.publish(closest_velocity);
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "motion_velocity_planner");
  MotionVelocityPlanner obj;

  ros::spin();

  return 0;
};

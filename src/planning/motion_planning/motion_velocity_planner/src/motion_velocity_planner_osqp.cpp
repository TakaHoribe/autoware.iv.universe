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

#define DEBUG_INFO(...) { if (show_debug_info_) {ROS_INFO(__VA_ARGS__); } }
#define DEBUG_WARN(...) { if (show_debug_info_) {ROS_WARN(__VA_ARGS__); } }
#define DEBUG_INFO_ALL(...) { if (show_debug_info_all_) {ROS_INFO(__VA_ARGS__); } }

MotionVelocityPlanner::MotionVelocityPlanner() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.param("max_velocity", planning_param_.max_velocity, double(20.0));    // 72.0 kmph
  pnh_.param("max_accel", planning_param_.max_accel, double(2.0));           // 0.11G
  pnh_.param("min_decel", planning_param_.min_decel, double(-3.0));          // -0.2G
  pnh_.param("max_lateral_accel", planning_param_.max_lat_acc, double(0.2)); //
  pnh_.param("replan_vel_deviation", planning_param_.replan_vel_deviation, double(3.0));
  pnh_.param("engage_velocity", planning_param_.engage_velocity, double(0.3));
  pnh_.param("engage_acceleration", planning_param_.engage_acceleration, double(0.1));
  pnh_.param("extract_ahead_dist", planning_param_.extract_ahead_dist, double(200.0));
  pnh_.param("extract_behind_dist", planning_param_.extract_behind_dist, double(3.0));
  pnh_.param("max_trajectory_length", planning_param_.max_trajectory_length, double(200.0));
  pnh_.param("min_trajectory_length", planning_param_.min_trajectory_length, double(30.0));
  pnh_.param("resample_total_time", planning_param_.resample_total_time, double(10.0));
  pnh_.param("resample_time_interval", planning_param_.resample_dt, double(0.1));
  pnh_.param("min_trajectory_distance_interval", planning_param_.min_trajectory_distance_interval, double(0.1));
  pnh_.param("stop_dist_not_to_drive_vehicle", planning_param_.stop_dist_not_to_drive_vehicle, double(1.5));
  pnh_.param("emergency_flag_vel_thr_kmph", planning_param_.emergency_flag_vel_thr_kmph, double(3.0));
  pnh_.param("stop_dist_mergin", planning_param_.stop_dist_mergin, double(0.55));

  pnh_.param("show_debug_info", show_debug_info_, bool(true));
  pnh_.param("show_debug_info_all", show_debug_info_all_, bool(false));
  pnh_.param("show_figure", show_figure_, bool(false));

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

  /* apply lateral acceleration filter */
  const double curvature_calc_dist = 3.0;
  const unsigned int idx_dist = std::max((int)(curvature_calc_dist / std::max(0.1, 0.001)), 1);
  autoware_planning_msgs::Trajectory base_traj_latacc_filtered;
  lateralAccelerationFilter(base_traj_extracted, planning_param_.max_lat_acc, idx_dist, /* out */ base_traj_latacc_filtered);


  /* (4) Resample extructed-waypoints with interpolation */
  autoware_planning_msgs::Trajectory base_traj_resampled;
  double ds = 0.0;
  if(!resampleTrajectory(base_traj_latacc_filtered, base_traj_resampled, ds))
  {
    return;
  }
  int base_traj_resampled_closest = vpu::calcClosestWaypoint(base_traj_resampled, current_pose_ptr_->pose);


  /* publish stop distance */
  publishStopDistance(base_traj_resampled, base_traj_resampled_closest);

  /* Change base velocity to zero when current_velocity == 0 & stop_dist is close */
  preventMoveToVeryCloseStopLine(base_traj_resampled_closest, planning_param_.stop_dist_not_to_drive_vehicle, base_traj_resampled);

  /* for negative velocity */
  const bool negative_velocity_flag =
      base_traj_resampled.points.at(base_traj_resampled_closest).twist.linear.x < 0.0 ? true : false;
  if (negative_velocity_flag == true)
  {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, /* out */ base_traj_resampled);
  }

  /* (5) Calculate the nearest point on the previously planned route (used to get initial planning speed) */
  int prev_output_trajectory_closest = -1;
  if (prev_output_trajectory_.points.size() != 0 /* if this is not initial planning */)
  {
    prev_output_trajectory_closest = vpu::calcClosestWaypoint(prev_output_trajectory_, current_pose_ptr_->pose);
  }
  DEBUG_INFO("[calcClosestWaypoint] for base_resampled : base_resampled.size() = %d, prev_planned_closest_ = %d",
             (int)base_traj_resampled.points.size(), prev_output_trajectory_closest);

  /* (6) Replan velocity */
  autoware_planning_msgs::Trajectory output_trajectory;
  replanVelocity(base_traj_resampled, base_traj_resampled_closest, prev_output_trajectory_, prev_output_trajectory_closest, ds, 
                /* out */ output_trajectory);
  DEBUG_INFO("[replanVelocity] : current_replanned.size() = %d", (int)output_trajectory.points.size());


  /* (7) max velocity filter for safety */
  vpu::maximumVelocityFilter(planning_param_.max_velocity, output_trajectory);

  /* for negative velocity */
  if (negative_velocity_flag == true)
  {
    vpu::multiplyConstantToTrajectoryVelocity(-1.0, output_trajectory);
  }

  /* publish message */
  output_trajectory.header = base_traj_raw_ptr_->header;
  publishTrajectory(output_trajectory);

  prev_output_trajectory_ = output_trajectory;


  /* for debug */
  MotionVelocityPlanner::publishClosestVelocity(output_trajectory.points.at(base_traj_resampled_closest).twist.linear.x);

  auto t_end = std::chrono::system_clock::now();
  double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(t_end - t_start).count() * 1.0e-6;
  DEBUG_INFO("run: calculation time = %f [ms]", elapsed_ms);
  DEBUG_INFO("============================== run() end ==============================\n\n");
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
                                         autoware_planning_msgs::Trajectory &output, double &ds) const
{
  std::vector<double> arclength;
  vpu::calcWaypointsArclength(input, arclength);
  const double smax = std::min(planning_param_.max_trajectory_length, arclength.back());
  const double Nt = planning_param_.resample_total_time / std::max(planning_param_.resample_dt, 0.001);
  ds = std::max(current_velocity_ptr_->twist.linear.x * planning_param_.resample_dt,
                planning_param_.min_trajectory_distance_interval);
  const double Ns = planning_param_.min_trajectory_length / std::max(ds, 0.001);
  const double N = std::max(Nt, Ns);
  std::vector<double> s_arr;
  double si = 0.0;
  for (int i = 0; i <= N; ++i)
  {
    s_arr.push_back(si);
    si += ds;
    if (si > smax)
    {
      break;
    }
  }
  if (!vpu::linearInterp1qTrajectory(arclength, input, s_arr, output))
  {
    ROS_WARN("[motion_velocity_planner]: fail trajectory interpolation. size : arclength = %lu, input = %lu, s_arr = %lu, output = %lu",
             arclength.size(), input.points.size(), s_arr.size(), output.points.size());
    return false;
  }
  return true;
}

void MotionVelocityPlanner::calcInitialMotion(const double &base_speed, const autoware_planning_msgs::Trajectory &base_waypoints,
                                        const int base_closest, const autoware_planning_msgs::Trajectory &prev_output,
                                        const int prev_replanned_traj_closest, MotionVelocityPlanner::Motion *initial_motion,
                                        int &init_type) const
{
  const double vehicle_speed = std::fabs(current_velocity_ptr_->twist.linear.x);

  if (prev_output.points.size() == 0 /* first time */)
  {
    initial_motion->vel = vehicle_speed;
    initial_motion->acc = 0.0;   // if possible, use actual vehicle acc & jerk value;
    initial_motion->jerk = 0.0;  // if possible, use actual vehicle acc & jerk value;
    init_type = 0;
    return;
  }

  const double desired_vel = prev_output.points.at(prev_replanned_traj_closest).twist.linear.x;
  const double vel_error = vehicle_speed - std::fabs(desired_vel);
  if (std::fabs(vel_error) > planning_param_.replan_vel_deviation /* when velocity tracking deviation is large */)
  {
    initial_motion->vel = vehicle_speed;  // use current vehicle speed
    initial_motion->acc = prev_output.points.at(prev_replanned_traj_closest).accel.linear.x;
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

    const double stop_dist = vpu::calcDist2d(base_waypoints.points.at(idx), base_waypoints.points.at(base_closest));
    if (!exist_stop_point || stop_dist > planning_param_.stop_dist_not_to_drive_vehicle)
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
  initial_motion->vel = prev_output.points.at(prev_replanned_traj_closest).twist.linear.x;
  initial_motion->acc = prev_output.points.at(prev_replanned_traj_closest).accel.linear.x;
  initial_motion->jerk = vpu::getTrajectoryJerk(prev_output, prev_replanned_traj_closest);
  init_type = 3;
  DEBUG_INFO("[calcInitialMotion]: normal update initial_motion.vel = %f, acc = %f, jerk = %f, vehicle_speed = %f, "
             "base_speed = %f",
             initial_motion->vel, initial_motion->acc, initial_motion->jerk, vehicle_speed, base_speed);
  return;
}

void MotionVelocityPlanner::optimizeVelocity(const Motion initial_motion, const autoware_planning_msgs::Trajectory &input, 
                                       const int closest, const double ds, autoware_planning_msgs::Trajectory &output)
{
  auto ts = std::chrono::system_clock::now();

  output = input;

  if ((int)input.points.size() < closest)
  {
    ROS_WARN("[MotionVelocityPlanner::optimizeVelocity] invalid closest.");
    return;
  }

  if (std::fabs(input.points.at(closest).twist.linear.x) < 0.1)
  {
    DEBUG_INFO("[MotionVelocityPlanner::optimizeVelocity] closest vmax < 0.1, keep stopping. return.");
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

  const double c = 1.0 / ds;

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
    const double cw = c * smooth_weight;
    P(i, i) += cw;
    P(i, i + 1) -= cw;
    P(i + 1, i) -= cw;
    P(i + 1, i + 1) += cw;
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
    A(i, j) = -c;
    A(i, j + 1) = c;
    A(i, j + N) = -2.0;
    upper_bound[i] = 0.0;
    lower_bound[i] = 0.0;
  }
  
  // initial condition
  const double v0 = initial_motion.vel;
  { 
    const unsigned int i = 3 * N - 1;
    A(i, 0) = 1.0;     // b0
    upper_bound[i] = v0 * v0;
    lower_bound[i] = v0 * v0;

    A(i + 1, N) = 1.0; // a0
    upper_bound[i + 1] = initial_motion.acc;
    lower_bound[i + 1] = initial_motion.acc;
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
    DEBUG_INFO_ALL("i = %d, v: %f, vmax: %f a: %f, b: %f, delta: %f, sigma: %f\n", i,std::sqrt(optval.at(i)), vmax[i], optval.at(i + N), optval.at(i), optval.at(i + 2*N), optval.at(i + 3*N));
  }


  auto tf2 = std::chrono::system_clock::now();
  double elapsed_ms2 = std::chrono::duration_cast<std::chrono::nanoseconds>(tf2 - ts2).count() * 1.0e-6;
  DEBUG_INFO("[optimization] initialization time = %f [ms], optimization time = %f [ms]", elapsed_ms1, elapsed_ms2);
};

void MotionVelocityPlanner::replanVelocity(const autoware_planning_msgs::Trajectory &input, const int input_closest,
                                     const autoware_planning_msgs::Trajectory &prev_output, const int prev_output_closest,
                                     const double ds, autoware_planning_msgs::Trajectory &output)
{
  const double base_speed = std::fabs(input.points.at(input_closest).twist.linear.x);

  /* calculate initial motion for planning */
  Motion init_m;
  int init_type = 3;
  calcInitialMotion(base_speed, input, input_closest, prev_output, prev_output_closest,
                                     /* out */ &init_m, /* out */ init_type);

  autoware_planning_msgs::Trajectory optimized_traj;
  optimizeVelocity(init_m, input, input_closest, ds, /* out */ optimized_traj);

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

#ifdef USE_MATPLOTLIB_FOR_VELOCITY_VIZ
  if (show_figure_)
    MotionVelocityPlanner::plotAll(stop_idx_zero_vel, input_closest, input, optimized_traj);
#endif
}

bool MotionVelocityPlanner::lateralAccelerationFilter(const autoware_planning_msgs::Trajectory &input,
                                                const double &max_lat_acc, const unsigned int curvature_calc_idx_dist,
                                                autoware_planning_msgs::Trajectory &output) const
{
  output = input; // initialize

  std::vector<double> curvature_v;
  vpu::calcTrajectoryCurvatureFrom3Points(input, curvature_calc_idx_dist, curvature_v);

  const double max_lat_acc_abs = std::fabs(max_lat_acc);

  for (unsigned int i = 0; i < input.points.size(); ++i)
  {
    const double curvature = std::max(std::fabs(curvature_v.at(i)), 0.0001 /* avoid 0 divide */);
    const double v_curvature_max = std::max(std::sqrt(max_lat_acc_abs / curvature), planning_param_.engage_velocity);
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
        vpu::zeroVelocity(trajectory);
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

#ifdef USE_MATPLOTLIB_FOR_VELOCITY_VIZ
void MotionVelocityPlanner::plotAll(const int &stop_idx_zero_vel, const int &input_closest, const autoware_planning_msgs::Trajectory &base,
                              const autoware_planning_msgs::Trajectory &optimized) const
{
  matplotlibcpp::clf();

  /* stop line */
  int stop_idx_plot[] = { stop_idx_zero_vel, stop_idx_zero_vel };
  int closest_idx_plot[] = { input_closest, input_closest };
  double y_plot1[] = { 0.0, 10.0 };
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::plot(stop_idx_plot, y_plot1, "k--");
  matplotlibcpp::plot(closest_idx_plot, y_plot1, "k");
  double y_plot2[] = { -2.0, 2.0 };
  matplotlibcpp::subplot(3, 1, 2);
  matplotlibcpp::plot(stop_idx_plot, y_plot2, "k--");
  matplotlibcpp::plot(closest_idx_plot, y_plot2, "k");
  double y_plot3[] = { -1.0, 1.0 };
  matplotlibcpp::subplot(3, 1, 3);
  matplotlibcpp::plot(stop_idx_plot, y_plot3, "k--");
  matplotlibcpp::plot(closest_idx_plot, y_plot3, "k");

  /* velocity */
  MotionVelocityPlanner::plotWaypoint(base, "r", "base");
  // MotionVelocityPlanner::plotWaypoint(latacc_filtered, "m", "latacc_filtered");
  MotionVelocityPlanner::plotWaypoint(optimized, "b", "optimized");
  MotionVelocityPlanner::plotAcceleration("c", optimized);
  MotionVelocityPlanner::plotJerk("m", optimized);

  std::vector<double> xv, yv;
  xv.push_back((double)input_closest);
  yv.push_back(std::fabs(current_velocity_ptr_->twist.linear.x));
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::plot(xv, yv, "k*");  // current vehicle velocity
  yv.clear();
  yv.push_back(std::fabs(optimized.points.at(input_closest).twist.linear.x));
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::plot(xv, yv, "bo");  // current planning initial velocity
  matplotlibcpp::ylim(0.0, 15.0);
  matplotlibcpp::pause(.01);  // plot all
}

void MotionVelocityPlanner::plotWaypoint(const autoware_planning_msgs::Trajectory &trajectory, const std::string &color_str,
                                   const std::string &label_str) const
{
  // std::vector<double> dist;
  // calcWaypointsArclength(trajectory, dist);
  
  std::vector<double> vec;
  for (const auto &wp : trajectory.points)
  {
    vec.push_back(wp.twist.linear.x);
  }
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::named_plot(label_str, vec, color_str);
}
void MotionVelocityPlanner::plotVelocity(const std::string &color_str, const autoware_planning_msgs::Trajectory &traj) const
{
  std::vector<double> vec;
  for (const auto &p : traj.points)
  {
    vec.push_back(p.twist.linear.x);
  }
  matplotlibcpp::subplot(3, 1, 1);
  matplotlibcpp::plot(vec, color_str);
}
void MotionVelocityPlanner::plotAcceleration(const std::string &color_str, const autoware_planning_msgs::Trajectory &traj) const
{
  std::vector<double> vec;
  for (const auto &p : traj.points)
  {
    vec.push_back(p.accel.linear.x);
  }
  matplotlibcpp::subplot(3, 1, 2);
  matplotlibcpp::plot(vec, color_str);
}

void MotionVelocityPlanner::plotJerk(const std::string &color_str, const autoware_planning_msgs::Trajectory &traj) const
{
  std::vector<double> vec;
  for (int i = 0; i < (int)traj.points.size(); ++i)
  {
    vec.push_back(vpu::getTrajectoryJerk(traj, i));
  }
  matplotlibcpp::subplot(3, 1, 3);
  matplotlibcpp::plot(vec, color_str);
}
#endif

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

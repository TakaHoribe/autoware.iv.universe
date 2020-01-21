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

#include "mpc_follower/mpc_follower_core.h"

#define DEBUG_INFO(...) { if (show_debug_info_) { ROS_INFO(__VA_ARGS__); }}

#define DEG2RAD 3.1415926535 / 180.0
#define RAD2DEG 180.0 / 3.1415926535

MPCFollower::MPCFollower()
    : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.param("show_debug_info", show_debug_info_, bool(false));
  pnh_.param("ctrl_period", ctrl_period_, double(0.03));
  pnh_.param("enable_path_smoothing", enable_path_smoothing_, bool(true));
  pnh_.param("enable_yaw_recalculation", enable_yaw_recalculation_, bool(false));
  pnh_.param("path_filter_moving_ave_num", path_filter_moving_ave_num_, int(35));
  pnh_.param("path_smoothing_times", path_smoothing_times_, int(1));
  pnh_.param("curvature_smoothing_num", curvature_smoothing_num_, int(35));
  pnh_.param("traj_resample_dist", traj_resample_dist_, double(0.1)); // [m]
  pnh_.param("admisible_position_error", admisible_position_error_, double(5.0));
  pnh_.param("admisible_yaw_error_deg", admisible_yaw_error_deg_, double(90.0));

  /* mpc parameters */
  pnh_.param("mpc_prediction_horizon", mpc_param_.prediction_horizon, int(70));
  pnh_.param("mpc_prediction_sampling_time", mpc_param_.prediction_sampling_time, double(0.1));
  pnh_.param("mpc_weight_lat_error", mpc_param_.weight_lat_error, double(1.0));
  pnh_.param("mpc_weight_heading_error", mpc_param_.weight_heading_error, double(0.0));
  pnh_.param("mpc_weight_heading_error_squared_vel_coeff", mpc_param_.weight_heading_error_squared_vel_coeff, double(0.3));
  pnh_.param("mpc_weight_steering_input", mpc_param_.weight_steering_input, double(1.0));
  pnh_.param("mpc_weight_steering_input_squared_vel_coeff", mpc_param_.weight_steering_input_squared_vel_coeff, double(0.25));
  pnh_.param("mpc_weight_lat_jerk", mpc_param_.weight_lat_jerk, double(0.0));
  pnh_.param("mpc_weight_terminal_lat_error", mpc_param_.weight_terminal_lat_error, double(1.0));
  pnh_.param("mpc_weight_terminal_heading_error", mpc_param_.weight_terminal_heading_error, double(0.1));
  pnh_.param("mpc_zero_ff_steer_deg", mpc_param_.zero_ff_steer_deg, double(2.0));
  pnh_.param("delay_compensation_time", mpc_param_.delay_compensation_time, double(0.0));

  pnh_.param("steer_lim_deg", steer_lim_deg_, double(35.0));
  pnh_.param("/vehicle_info/wheel_base", wheelbase_, double(2.9));

  /* vehicle model setup */
  pnh_.param("vehicle_model_type", vehicle_model_type_, std::string("kinematics"));
  if (vehicle_model_type_ == "kinematics")
  {
    double steer_tau;
    pnh_.param("vehicle_model_steer_tau", steer_tau, double(0.1));

    vehicle_model_ptr_ = std::make_shared<KinematicsBicycleModel>(wheelbase_, steer_lim_deg_ * DEG2RAD, steer_tau);
    ROS_INFO("[MPC] set vehicle_model = kinematics");
  }
  else if (vehicle_model_type_ == "kinematics_no_delay")
  {
    vehicle_model_ptr_ = std::make_shared<KinematicsBicycleModelNoDelay>(wheelbase_, steer_lim_deg_ * DEG2RAD);
    ROS_INFO("[MPC] set vehicle_model = kinematics_no_delay");
  }
  else if (vehicle_model_type_ == "dynamics")
  {
    double mass_fl, mass_fr, mass_rl, mass_rr, cf, cr;
    pnh_.param("mass_fl", mass_fl, double(600));
    pnh_.param("mass_fr", mass_fr, double(600));
    pnh_.param("mass_rl", mass_rl, double(600));
    pnh_.param("mass_rr", mass_rr, double(600));
    pnh_.param("cf", cf, double(155494.663));
    pnh_.param("cr", cr, double(155494.663));

    vehicle_model_ptr_ = std::make_shared<DynamicsBicycleModel>(wheelbase_, mass_fl, mass_fr, mass_rl, mass_rr, cf, cr);
    ROS_INFO("[MPC] set vehicle_model = dynamics");
  }
  else
  {
    ROS_ERROR("[MPC] vehicle_model_type is undefined");
  }

  /* QP solver setup */
  std::string qp_solver_type_;
  pnh_.param("qp_solver_type", qp_solver_type_, std::string("unconstraint_fast"));
  if (qp_solver_type_ == "unconstraint_fast")
  {
    qpsolver_ptr_ = std::make_shared<QPSolverEigenLeastSquareLLT>();
    ROS_INFO("[MPC] set qp solver = unconstraint_fast");
  }
  else if (qp_solver_type_ == "qpoases_hotstart")
  {
    // int max_iter;
    // pnh_.param("qpoases_max_iter", max_iter, int(500));
    // qpsolver_ptr_ = std::make_shared<QPSolverQpoasesHotstart>(max_iter);
    // ROS_INFO("[MPC] set qp solver = qpoases_hotstart");
  }
  else
  {
    ROS_ERROR("[MPC] qp_solver_type is undefined");
  }

  steer_cmd_prev_ = 0.0;
  lateral_error_prev_ = 0.0;
  yaw_error_prev_ = 0.0;

  /* delay compensation */
  const int delay_step = std::round(mpc_param_.delay_compensation_time / ctrl_period_);
  std::deque<double> tmp_deque(delay_step, 0.0);
  input_buffer_ = tmp_deque;

  /* initialize lowpass filter */
  double steering_lpf_cutoff_hz, error_deriv_lpf_curoff_hz;
  pnh_.param("steering_lpf_cutoff_hz", steering_lpf_cutoff_hz, double(3.0));
  pnh_.param("error_deriv_lpf_curoff_hz", error_deriv_lpf_curoff_hz, double(5.0));
  lpf_steering_cmd_.initialize(ctrl_period_, steering_lpf_cutoff_hz);
  lpf_lateral_error_.initialize(ctrl_period_, error_deriv_lpf_curoff_hz);
  lpf_yaw_error_.initialize(ctrl_period_, error_deriv_lpf_curoff_hz);

  /* set up ros system */
  timer_control_ = nh_.createTimer(ros::Duration(ctrl_period_), &MPCFollower::timerCallback, this);
  pub_twist_cmd_ = pnh_.advertise<geometry_msgs::TwistStamped>("output/twist_raw", 1);
  pub_steer_vel_ctrl_cmd_ = pnh_.advertise<autoware_control_msgs::ControlCommandStamped>("output/control_raw", 1);
  sub_ref_path_ = pnh_.subscribe("input/reference_trajectory", 1, &MPCFollower::callbackRefPath, this);
  sub_current_vel_ = pnh_.subscribe("input/current_velocity", 1, &MPCFollower::callbackCurrentVelocity, this);
  sub_vehicle_status_ = pnh_.subscribe("input/vehicle_status", 1, &MPCFollower::callbackVehicleStatus, this);

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
      ROS_INFO("[mpc_follower] is waitting to get map to base_link transform. %s", ex.what());
      continue;
    }
  }

  /* for debug */
  pub_debug_marker_ = pnh_.advertise<visualization_msgs::MarkerArray>("debug/markers", 1);
  pub_debug_mpc_calc_time_ = pnh_.advertise<std_msgs::Float32>("debug/mpc_calc_time", 1);

  pub_debug_values_ = pnh_.advertise<std_msgs::Float32MultiArray>("debug/debug_values", 1);
  sub_estimate_twist_ = nh_.subscribe("estimate_twist", 1, &MPCFollower::callbackEstimateTwist, this);
};

void MPCFollower::timerCallback(const ros::TimerEvent &te)
{
  updateCurrentPose();

  /* guard */
  if (vehicle_model_ptr_ == nullptr || qpsolver_ptr_ == nullptr)
  {
    DEBUG_INFO("[MPC] vehicle_model = %d, qp_solver = %d", !(vehicle_model_ptr_ == nullptr), !(qpsolver_ptr_ == nullptr));
    publishCtrlCmd(0.0, 0.0, steer_cmd_prev_, 0.0); // publish brake
    return;
  }

  if (ref_traj_.size() == 0 || current_pose_ptr_ == nullptr || current_velocity_ptr_ == nullptr || current_steer_ptr_ == nullptr)
  {
    DEBUG_INFO("[MPC] MPC is not solved. ref_traj_.size() = %d, pose = %d,  velocity = %d,  steer = %d",
               ref_traj_.size(), current_pose_ptr_ != nullptr, current_velocity_ptr_ != nullptr, current_steer_ptr_ != nullptr);

    publishCtrlCmd(0.0, 0.0, steer_cmd_prev_, 0.0); // publish brake
    return;
  }

  /* control command */
  double vel_cmd = 0.0;
  double acc_cmd = 0.0;
  double steer_cmd = 0.0;
  double steer_vel_cmd = 0.0;

  /* solve MPC */
  auto start = std::chrono::system_clock::now();
  const bool mpc_solved = calculateMPC(vel_cmd, acc_cmd, steer_cmd, steer_vel_cmd);
  double elapsed_ms = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() * 1.0e-6;
  DEBUG_INFO("[MPC] timerCallback: MPC calculating time = %f [ms]\n", elapsed_ms);

  /* publish computing time */
  std_msgs::Float32 mpc_calc_time_msg;
  mpc_calc_time_msg.data = elapsed_ms;
  pub_debug_mpc_calc_time_.publish(mpc_calc_time_msg);

  if (!mpc_solved)
  {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] MPC is not solved. publish 0 velocity.");
    vel_cmd = 0.0;
    acc_cmd = 0.0;
    steer_cmd = steer_cmd_prev_;
    steer_vel_cmd = 0.0;
  }

  publishCtrlCmd(vel_cmd, acc_cmd, steer_cmd, steer_vel_cmd);

};

bool MPCFollower::calculateMPC(double &vel_cmd, double &acc_cmd, double &steer_cmd, double &steer_vel_cmd)
{
  const int N = mpc_param_.prediction_horizon;
  const double DT = mpc_param_.prediction_sampling_time;
  const int DIM_X = vehicle_model_ptr_->getDimX();
  const int DIM_U = vehicle_model_ptr_->getDimU();
  const int DIM_Y = vehicle_model_ptr_->getDimY();

  const double current_yaw = tf2::getYaw(current_pose_ptr_->pose.orientation);

  /* calculate nearest point on reference trajectory (used as initial state) */
  unsigned int nearest_index = 0;
  double yaw_err, dist_err, nearest_traj_time;
  geometry_msgs::Pose nearest_pose;
  if (!MPCUtils::calcNearestPoseInterp(ref_traj_, current_pose_ptr_->pose, nearest_pose, nearest_index, dist_err, yaw_err, nearest_traj_time))
  {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] calculateMPC: error in calculating nearest pose. stop mpc.");
    return false;
  };

  /* check if lateral error is not too large */
  if (dist_err > admisible_position_error_ || std::fabs(yaw_err) > DEG2RAD * admisible_yaw_error_deg_ )
  {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] error is over limit, stop mpc. (pos: error = %f[m], limit: %f[m], yaw: error = %f[deg], limit %f[deg])",
             dist_err, admisible_position_error_, RAD2DEG * (yaw_err), admisible_yaw_error_deg_);
    return false;
  }

  /* set mpc initial time */
  const double mpc_start_time = nearest_traj_time;

  /* check trajectory length */
  const double mpc_end_time = mpc_start_time + (N - 1) * DT + mpc_param_.delay_compensation_time + ctrl_period_;
  if (mpc_end_time > ref_traj_.relative_time.back())
  {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] path is too short for prediction. path end: %f[s], mpc end time: %f[s]", ref_traj_.relative_time.back(), mpc_end_time);
    return false;
  }

  /* convert tracking x,y error to lat error */
  const double err_x = current_pose_ptr_->pose.position.x - nearest_pose.position.x;
  const double err_y = current_pose_ptr_->pose.position.y - nearest_pose.position.y;
  const double sp_yaw = tf2::getYaw(nearest_pose.orientation);
  const double err_lat = -std::sin(sp_yaw) * err_x + std::cos(sp_yaw) * err_y;

  /* get steering angle */
  const double steer = *current_steer_ptr_;

  /* define initial state for error dynamics */
  Eigen::VectorXd x0 = Eigen::VectorXd::Zero(DIM_X);
  if (vehicle_model_type_ == "kinematics")
  {
    x0 << err_lat, yaw_err, steer;
  }
  else if (vehicle_model_type_ == "kinematics_no_delay")
  {
    x0 << err_lat, yaw_err;
  }
  else if (vehicle_model_type_ == "dynamics")
  {
    double dot_err_lat = (err_lat - lateral_error_prev_) / ctrl_period_;
    double dot_err_yaw = (yaw_err - yaw_error_prev_) / ctrl_period_;
    DEBUG_INFO("[MPC] (before lpf) dot_err_lat = %f, dot_err_yaw = %f", dot_err_lat, dot_err_yaw);
    lateral_error_prev_ = err_lat;
    yaw_error_prev_ = yaw_err;
    dot_err_lat = lpf_lateral_error_.filter(dot_err_lat);
    dot_err_yaw = lpf_yaw_error_.filter(dot_err_yaw);
    DEBUG_INFO("[MPC] (after lpf) dot_err_lat = %f, dot_err_yaw = %f", dot_err_lat, dot_err_yaw);
    x0 << err_lat, dot_err_lat, yaw_err, dot_err_yaw;
  }
  else
  {
    ROS_ERROR("vehicle_model_type is undefined");
    return false;
  }
  DEBUG_INFO("[MPC] selfpose.x = %f, y = %f, yaw = %f", current_pose_ptr_->pose.position.x, current_pose_ptr_->pose.position.y, current_yaw);
  DEBUG_INFO("[MPC] nearpose.x = %f, y = %f, yaw = %f", nearest_pose.position.x, nearest_pose.position.y, tf2::getYaw(nearest_pose.orientation));
  DEBUG_INFO("[MPC] nearest_index = %d, nearest_traj_time = %f", nearest_index, nearest_traj_time);
  DEBUG_INFO("[MPC] lat error = %f, yaw error = %f, steer = %f, sp_yaw = %f, my_yaw = %f", err_lat, yaw_err, steer, sp_yaw, current_yaw);


  /////////////// delay compensation  ///////////////
  Eigen::MatrixXd Ad(DIM_X, DIM_X);
  Eigen::MatrixXd Bd(DIM_X, DIM_U);
  Eigen::MatrixXd Wd(DIM_X, 1);
  Eigen::MatrixXd Cd(DIM_Y, DIM_X);
  Eigen::MatrixXd Uref(DIM_U, 1);

  Eigen::MatrixXd x_curr = x0;
  double mpc_curr_time = mpc_start_time;
  for (unsigned int i = 0; i < input_buffer_.size(); ++i)
  {
    double k = 0.0;
    double v = 0.0;
    if (!MPCUtils::interp1d(ref_traj_.relative_time, ref_traj_.k, mpc_curr_time, k) ||
        !MPCUtils::interp1d(ref_traj_.relative_time, ref_traj_.vx, mpc_curr_time, v))
    {
      ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] calculateMPC: mpc resample error at delay compensation, stop mpc calculation. check code!");
      return false;
    }

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(v);
    vehicle_model_ptr_->setCurvature(k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, ctrl_period_);
    Eigen::MatrixXd ud = Eigen::MatrixXd::Zero(DIM_U, 1);
    ud(0, 0) = input_buffer_.at(i); // for steering input delay
    x_curr = Ad * x_curr + Bd * ud + Wd;
    mpc_curr_time += ctrl_period_;
  }
  x0 = x_curr; // set delay compensated initial state


  /////////////// generate mpc matrix  ///////////////
  /*
   * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
   * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * Rex * (Uex - Urefex)
   * Qex = diag([Q,Q,...]), Rex = diag([R,R,...])
   */

  Eigen::MatrixXd Aex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_X);
  Eigen::MatrixXd Bex = Eigen::MatrixXd::Zero(DIM_X * N, DIM_U * N);
  Eigen::MatrixXd Wex = Eigen::MatrixXd::Zero(DIM_X * N, 1);
  Eigen::MatrixXd Cex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_X * N);
  Eigen::MatrixXd Qex = Eigen::MatrixXd::Zero(DIM_Y * N, DIM_Y * N);
  Eigen::MatrixXd Rex = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd Urefex = Eigen::MatrixXd::Zero(DIM_U * N, 1);

  /* weight matrix depends on the vehicle model */
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Eigen::MatrixXd Q_adaptive = Eigen::MatrixXd::Zero(DIM_Y, DIM_Y);
  Eigen::MatrixXd R_adaptive = Eigen::MatrixXd::Zero(DIM_U, DIM_U);
  Q(0, 0) = mpc_param_.weight_lat_error;
  Q(1, 1) = mpc_param_.weight_heading_error;
  R(0, 0) = mpc_param_.weight_steering_input;

  /* resample ref_traj with mpc sampling time */
  std::vector<double> mpc_time_v;
  for (int i = 0; i < N; ++i)
  {
    mpc_time_v.push_back(mpc_curr_time + i * DT);
  }
  MPCTrajectory mpc_resampled_ref_traj;
  if (!MPCUtils::interp1dMPCTraj(ref_traj_.relative_time, ref_traj_, mpc_time_v, mpc_resampled_ref_traj))
  {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] calculateMPC: mpc resample error, stop mpc calculation. check code!");
    return false;
  }

  /* predict dynamics for N times */
  for (int i = 0; i < N; ++i)
  {
    const double sign_vx = mpc_resampled_ref_traj.vx[i] > 0 ? 1 : (mpc_resampled_ref_traj.vx[i] < 0 ? -1 : 0);
    const double ref_k = mpc_resampled_ref_traj.k[i] * sign_vx;
    const double ref_vx = mpc_resampled_ref_traj.vx[i];
    const double ref_vx_squared = ref_vx * ref_vx;

    /* get discrete state matrix A, B, C, W */
    vehicle_model_ptr_->setVelocity(ref_vx);
    vehicle_model_ptr_->setCurvature(ref_k);
    vehicle_model_ptr_->calculateDiscreteMatrix(Ad, Bd, Cd, Wd, DT);

    Q_adaptive = Q;
    R_adaptive = R;
    if (i == N - 1)
    {
      Q_adaptive(0, 0) = mpc_param_.weight_terminal_lat_error;
      Q_adaptive(1, 1) = mpc_param_.weight_terminal_heading_error;
    }
    Q_adaptive(1, 1) += ref_vx_squared * mpc_param_.weight_heading_error_squared_vel_coeff;
    R_adaptive(0, 0) += ref_vx_squared * mpc_param_.weight_steering_input_squared_vel_coeff;

    /* update mpc matrix */
    int idx_x_i = i * DIM_X;
    int idx_x_i_prev = (i - 1) * DIM_X;
    int idx_u_i = i * DIM_U;
    int idx_y_i = i * DIM_Y;
    if (i == 0)
    {
      Aex.block(0, 0, DIM_X, DIM_X) = Ad;
      Bex.block(0, 0, DIM_X, DIM_U) = Bd;
      Wex.block(0, 0, DIM_X, 1) = Wd;
    }
    else
    {
      Aex.block(idx_x_i, 0, DIM_X, DIM_X) = Ad * Aex.block(idx_x_i_prev, 0, DIM_X, DIM_X);
      for (int j = 0; j < i; ++j)
      {
        int idx_u_j = j * DIM_U;
        Bex.block(idx_x_i, idx_u_j, DIM_X, DIM_U) = Ad * Bex.block(idx_x_i_prev, idx_u_j, DIM_X, DIM_U);
      }
      Wex.block(idx_x_i, 0, DIM_X, 1) = Ad * Wex.block(idx_x_i_prev, 0, DIM_X, 1) + Wd;
    }
    Bex.block(idx_x_i, idx_u_i, DIM_X, DIM_U) = Bd;
    Cex.block(idx_y_i, idx_x_i, DIM_Y, DIM_X) = Cd;
    Qex.block(idx_y_i, idx_y_i, DIM_Y, DIM_Y) = Q_adaptive;
    Rex.block(idx_u_i, idx_u_i, DIM_U, DIM_U) = R_adaptive;

    /* get reference input (feed-forward) */
    vehicle_model_ptr_->calculateReferenceInput(Uref);
    if (std::fabs(Uref(0, 0)) < DEG2RAD * mpc_param_.zero_ff_steer_deg)
    {
      Uref(0, 0) = 0.0; // ignore curvature noise
    }

    Urefex.block(i * DIM_U, 0, DIM_U, 1) = Uref;

    mpc_curr_time += DT;
  }

  /* add lateral jerk : weight for (v * {u(i) - u(i-1)} )^2 */
  for (int i = 0; i < N - 1; ++i)
  {
    const double v = mpc_resampled_ref_traj.vx[i];
    const double lateral_jerk_weight = v * v * mpc_param_.weight_lat_jerk;
    Rex(i, i) += lateral_jerk_weight;
    Rex(i + 1, i) -= lateral_jerk_weight;
    Rex(i, i + 1) -= lateral_jerk_weight;
    Rex(i + 1, i + 1) += lateral_jerk_weight;
  }

  if (Aex.array().isNaN().any() || Bex.array().isNaN().any() ||
      Cex.array().isNaN().any() || Wex.array().isNaN().any())
  {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] calculateMPC: model matrix includes NaN, stop MPC.");
    return false;
  }

  /////////////// optimization ///////////////
  /*
   * solve quadratic optimization.
   * cost function: 1/2 * Uex' * H * Uex + f' * Uex
   */
  const Eigen::MatrixXd CB = Cex * Bex;
  const Eigen::MatrixXd QCB = Qex * CB;
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  H.triangularView<Eigen::Upper>() = CB.transpose() * QCB; // NOTE: This calculation is very heavy. searching for a good way...
  H.triangularView<Eigen::Upper>() += Rex;
  H.triangularView<Eigen::Lower>() = H.transpose();
  Eigen::MatrixXd f = (Cex * (Aex * x0 + Wex)).transpose() * QCB - Urefex.transpose() * Rex;

  /* constraint matrix : lb < U < ub, lbA < A*U < ubA */
  const double u_lim = DEG2RAD * steer_lim_deg_;
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(DIM_U * N, DIM_U * N);
  Eigen::MatrixXd lbA = Eigen::MatrixXd::Zero(DIM_U * N, 1);
  Eigen::MatrixXd ubA = Eigen::MatrixXd::Zero(DIM_U * N, 1);
  Eigen::VectorXd lb = Eigen::VectorXd::Constant(DIM_U * N, -u_lim); // min steering angle
  Eigen::VectorXd ub = Eigen::VectorXd::Constant(DIM_U * N, u_lim);  // max steering angle

  auto start = std::chrono::system_clock::now();
  Eigen::VectorXd Uex;
  if (!qpsolver_ptr_->solve(H, f.transpose(), A, lb, ub, lbA, ubA, Uex))
  {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] qp solver error");
    return false;
  }
  double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now() - start).count() * 1.0e-6;
  DEBUG_INFO("[MPC] calculateMPC: qp solver calculation time = %f [ms]", elapsed);

  if (Uex.array().isNaN().any())
  {
    ROS_WARN_DELAYED_THROTTLE(5.0, "[MPC] calculateMPC: model Uex includes NaN, stop MPC. ");
    return false;
  }

  /* saturation */
  const double u_sat = std::max(std::min(Uex(0), u_lim), -u_lim);

  /* filtering */
  const double u_filtered = lpf_steering_cmd_.filter(u_sat);

  /* set steering command */
  steer_cmd = u_filtered;
  steer_vel_cmd = (Uex(1) - Uex(0)) / DT;

  /* Velocity control: for simplicity, now we calculate steer and speed separately */
  vel_cmd = ref_traj_.vx[nearest_index];
  acc_cmd = (ref_traj_.vx[nearest_index] - ref_traj_.vx[std::max(0, (int)nearest_index - 1)]) / DT;

  steer_cmd_prev_ = steer_cmd;

  /* save to buffer */
  input_buffer_.push_back(steer_cmd);
  input_buffer_.pop_front();

  DEBUG_INFO("[MPC] calculateMPC: mpc steer command raw = %f, filtered = %f, steer_vel_cmd = %f", Uex(0, 0), u_filtered, steer_vel_cmd);
  DEBUG_INFO("[MPC] calculateMPC: mpc vel command = %f, acc_cmd = %f", vel_cmd, acc_cmd);

  ////////////////// DEBUG ///////////////////

  /* calculate predicted trajectory */
  Eigen::VectorXd Xex = Aex * x0 + Bex * Uex + Wex;
  MPCTrajectory debug_mpc_predicted_traj;
  for (int i = 0; i < N; ++i)
  {
    const double lat_error = Xex(i * DIM_X);
    const double yaw_error = Xex(i * DIM_X + 1);
    const double x = mpc_resampled_ref_traj.x[i] - std::sin(mpc_resampled_ref_traj.yaw[i]) * lat_error;
    const double y = mpc_resampled_ref_traj.y[i] + std::cos(mpc_resampled_ref_traj.yaw[i]) * lat_error;
    const double z = mpc_resampled_ref_traj.z[i];
    debug_mpc_predicted_traj.push_back(x, y, z, mpc_resampled_ref_traj.yaw[i] + yaw_error, 0, 0, 0);
  }

  /* publish for visualization */
  visualization_msgs::MarkerArray markers;
  convertTrajToMarker(debug_mpc_predicted_traj, markers, "predicted_trajectory", 0.99, 0.99, 0.99, 0.2);
  pub_debug_marker_.publish(markers);

  /* publish debug values */
  {
    double curr_v = current_velocity_ptr_->twist.linear.x;
    double nearest_k = 0.0;
    MPCUtils::interp1d(ref_traj_.relative_time, ref_traj_.k, nearest_traj_time, nearest_k);

    std_msgs::Float32MultiArray debug_values;
    debug_values.data.push_back(steer_cmd);                                      // [0] final steering command (MPC + LPF)
    debug_values.data.push_back(u_sat);                                          // [1] mpc calculation result
    debug_values.data.push_back(Urefex(0));                                      // [2] feedforward steering value
    debug_values.data.push_back(std::atan(nearest_k * wheelbase_));              // [3] feedforward steering value raw
    debug_values.data.push_back(steer);                                          // [4] current steering angle
    debug_values.data.push_back(err_lat);                                        // [5] lateral error
    debug_values.data.push_back(tf2::getYaw(current_pose_ptr_->pose.orientation));  // [6] current_pose yaw
    debug_values.data.push_back(tf2::getYaw(nearest_pose.orientation));          // [7] nearest_pose yaw
    debug_values.data.push_back(yaw_err);                                        // [8] yaw error
    debug_values.data.push_back(vel_cmd);                                        // [9] command velocitys
    debug_values.data.push_back(current_velocity_ptr_->twist.linear.x);                 // [10] measured velocity
    debug_values.data.push_back(curr_v * tan(steer_cmd) / wheelbase_);  // [11] angvel from steer comand (MPC assumes)
    debug_values.data.push_back(curr_v * tan(steer) / wheelbase_);      // [12] angvel from measured steer
    debug_values.data.push_back(curr_v * nearest_k);                    // [13] angvel from path curvature (Path angvel)
    debug_values.data.push_back(nearest_k);                             // [14] nearest path curvature
    debug_values.data.push_back(estimate_twist_.twist.linear.x);        // [15] current velocity
    debug_values.data.push_back(estimate_twist_.twist.angular.z);       // [16] estimate twist angular velocity (real angvel)
    pub_debug_values_.publish(debug_values);
  }

  return true;
};

void MPCFollower::callbackRefPath(const autoware_planning_msgs::Trajectory::ConstPtr &msg)
{
  if (msg->points.size() < 3)
  {
    DEBUG_INFO("[MPC] received path size is < 3, not enough.");
    return;
  }

  current_trajectory_ = *msg;
  DEBUG_INFO("[MPC] trajectory callback: received trajectory size = %lu", current_trajectory_.points.size());

  MPCTrajectory traj;

  /* calculate relative time */
  std::vector<double> relative_time;
  MPCUtils::calcPathRelativeTime(current_trajectory_, relative_time);
  DEBUG_INFO("[MPC] path callback: relative_time.size() = %lu, front() = %f, back() = %f",
             relative_time.size(), relative_time.front(), relative_time.back());

  /* resampling */
  MPCUtils::convertWaypointsToMPCTrajWithDistanceResample(current_trajectory_, relative_time, traj_resample_dist_, traj);
  MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  DEBUG_INFO("[MPC] path callback: resampled traj size() = %lu", traj.relative_time.size());


  /* path smoothing */
  if (enable_path_smoothing_ && (int)traj.size() > 2 * path_filter_moving_ave_num_)
  {
    MPCTrajectory traj_smoothed = traj;
    for (int i = 0; i < path_smoothing_times_; ++i)
    {
      if (!MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj_smoothed.x) ||
          !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj_smoothed.y) ||
          !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj_smoothed.yaw) ||
          !MoveAverageFilter::filt_vector(path_filter_moving_ave_num_, traj_smoothed.vx))
      {
        DEBUG_INFO("[MPC] path callback: filtering error. ignore this trajectory");
        break;
      }
      else
      {
        traj = traj_smoothed;
      }
    }
  }


  /* calculate yaw angle */
  if (enable_yaw_recalculation_)
  {
    MPCUtils::calcTrajectoryYawFromXY(traj);
    MPCUtils::convertEulerAngleToMonotonic(traj.yaw);
  }
  /* calculate curvature */
  MPCUtils::calcTrajectoryCurvature(traj, curvature_smoothing_num_);
  const double max_k = *max_element(traj.k.begin(), traj.k.end());
  const double min_k = *min_element(traj.k.begin(), traj.k.end());
  DEBUG_INFO("[MPC] path callback: trajectory curvature : max_k = %f, min_k = %f", max_k, min_k);
  /* add end point with vel=0 on traj for mpc prediction */
  const double mpc_predict_time_length = (mpc_param_.prediction_horizon + 1) * mpc_param_.prediction_sampling_time + mpc_param_.delay_compensation_time + ctrl_period_;
  const double end_velocity = 0.0;
  traj.vx.back() = end_velocity; // also for end point
  traj.push_back(traj.x.back(), traj.y.back(), traj.z.back(), traj.yaw.back(),
                 end_velocity, traj.k.back(), traj.relative_time.back() + mpc_predict_time_length);

  if (!traj.size())
  {
    DEBUG_INFO("[MPC] path callback: trajectory size is undesired.");
    DEBUG_INFO("size: x=%lu, y=%lu, z=%lu, yaw=%lu, v=%lu,k=%lu,t=%lu", traj.x.size(), traj.y.size(),
               traj.z.size(), traj.yaw.size(), traj.vx.size(), traj.k.size(), traj.relative_time.size());
    return;
  }

  ref_traj_ = traj;

  /* publish trajectory for visualize */
  visualization_msgs::MarkerArray markers;
  convertTrajToMarker(ref_traj_, markers, "filtered_reference_trajectory", 0.0, 0.5, 1.0, 0.05);
  pub_debug_marker_.publish(markers);
};

void MPCFollower::updateCurrentPose()
{

  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform("map", "base_link", ros::Time(0));
  }
  catch (tf2::TransformException &ex)
  {
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
};

void MPCFollower::callbackVehicleStatus(const autoware_control_msgs::VehicleStatusStamped::ConstPtr &msg)
{
  current_steer_ptr_ = std::make_shared<double>(msg->status.steering_angle);
};

void MPCFollower::callbackCurrentVelocity(const geometry_msgs::TwistStamped::ConstPtr &msg)
{
  current_velocity_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(*msg);
};

void MPCFollower::publishCtrlCmd(const double &vel_cmd, const double &acc_cmd, const double &steer_cmd, const double &steer_vel_cmd)
{
  autoware_control_msgs::ControlCommandStamped cmd;
  cmd.header.frame_id = "base_link";
  cmd.header.stamp = ros::Time::now();
  cmd.control.velocity = vel_cmd;
  cmd.control.acceleration = acc_cmd;
  cmd.control.steering_angle = steer_cmd;
  cmd.control.steering_angle_velocity = steer_vel_cmd;
  pub_steer_vel_ctrl_cmd_.publish(cmd);
}

MPCFollower::~MPCFollower()
{
  ROS_INFO("Publish 0 twist before I died.");
  double vel_cmd = 0.0;
  double acc_cmd = 0.0;
  double steer_cmd = 0.0;
  double steer_vel_cmd = 0.0;
  if (current_steer_ptr_ != nullptr)
    steer_cmd = *current_steer_ptr_;
  publishCtrlCmd(vel_cmd, acc_cmd, steer_cmd, steer_vel_cmd);
};

void MPCFollower::convertTrajToMarker(const MPCTrajectory &traj, visualization_msgs::MarkerArray &markers,
                                      std::string ns, double r, double g, double b, double z)
{
  visualization_msgs::Marker marker;
  marker.header.frame_id = current_trajectory_.header.frame_id;
  marker.header.stamp = ros::Time();
  marker.ns = ns + "/line";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_STRIP;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.15;
  marker.color.a = 0.9;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.pose.orientation.w = 1.0;
  for (unsigned int i = 0; i < traj.x.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = traj.x.at(i);
    p.y = traj.y.at(i);
    p.z = traj.z.at(i) + z;
    marker.points.push_back(p);
  }
  markers.markers.push_back(marker);

  visualization_msgs::Marker marker_poses;
  for (unsigned int i = 0; i < traj.size(); ++i)
  {
    marker_poses.header.frame_id = current_trajectory_.header.frame_id;
    marker_poses.header.stamp = ros::Time();
    marker_poses.ns = ns + "/poses";
    marker_poses.id = i;
    marker_poses.lifetime = ros::Duration(0.5);
    marker_poses.type = visualization_msgs::Marker::ARROW;
    marker_poses.action = visualization_msgs::Marker::ADD;
    marker_poses.pose.position.x = traj.x.at(i);
    marker_poses.pose.position.y = traj.y.at(i);
    marker_poses.pose.position.z = traj.z.at(i);
    marker_poses.pose.orientation = MPCUtils::getQuaternionFromYaw(traj.yaw.at(i));
    marker_poses.scale.x = 0.1;
    marker_poses.scale.y = 0.05;
    marker_poses.scale.z = 0.1;
    marker_poses.color.a = 0.99; // Don't forget to set the alpha!
    marker_poses.color.r = r;
    marker_poses.color.g = g;
    marker_poses.color.b = b;
    markers.markers.push_back(marker_poses);
  }

  visualization_msgs::Marker marker_text;
  marker_text.header.frame_id = current_trajectory_.header.frame_id;
  marker_text.header.stamp = ros::Time();
  marker_text.ns = ns + "/text";
  marker_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_text.action = visualization_msgs::Marker::ADD;
  marker_text.scale.z = 0.2;
  marker_text.color.a = 0.99; // Don't forget to set the alpha!
  marker_text.color.r = r;
  marker_text.color.g = g;
  marker_text.color.b = b;

  for (unsigned int i = 0; i < traj.size(); ++i)
  {
    marker_text.id = i;
    marker_text.pose.position.x = traj.x.at(i);
    marker_text.pose.position.y = traj.y.at(i);
    marker_text.pose.position.z = traj.z.at(i);
    marker_text.pose.orientation = MPCUtils::getQuaternionFromYaw(traj.yaw.at(i));
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(1) << traj.vx.at(i) << ", " <<  i;
    marker_text.text = oss.str();
    markers.markers.push_back(marker_text);
  }
}

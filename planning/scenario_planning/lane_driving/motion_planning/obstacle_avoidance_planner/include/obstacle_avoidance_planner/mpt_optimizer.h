/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
#ifndef MPTOPTIMIZER_H
#define MPTOPTIMIZER_H

#include <autoware_planning_msgs/TrajectoryPoint.h>

namespace cv
{
class Mat;
}

namespace osqp
{
class OSQPInterface;
}

struct DebugData;
struct TrajectoryParam;
struct QPParam;
struct ConstrainParam;
struct VehicleParam;
struct Rectangle;
struct CVMaps;
struct Trajectories;

class VehicleModelInterface;

struct ReferencePoint
{
  geometry_msgs::Point p;
  double k = 0;
  double v = 0;
  double yaw = 0;
  geometry_msgs::Quaternion q;
  double s = 0;
  geometry_msgs::Pose top_pose;
  geometry_msgs::Pose mid_pose;
  double delta_yaw_from_p1;
  double delta_yaw_from_p2;
  bool is_fix = false;
  double fixing_lat;
};

struct Bounds
{
  struct SingleBounds
  {
    SingleBounds & operator=(std::vector<double> & bounds)
    {
      ub = bounds[0];
      lb = bounds[1];
    }
    double ub;  // left
    double lb;  // right
  } c0, c1, c2;
};

struct MPTMatrix
{
  Eigen::MatrixXd Aex;
  Eigen::MatrixXd Bex;
  Eigen::MatrixXd Wex;
  Eigen::MatrixXd Cex;
  Eigen::MatrixXd Qex;
  Eigen::MatrixXd R1ex;
  Eigen::MatrixXd R2ex;
  Eigen::MatrixXd Urefex;
};

struct ObjectiveMatrix
{
  Eigen::MatrixXd hessian;
  std::vector<double> gradient;
};

struct ConstraintMatrix
{
  Eigen::MatrixXd linear;
  std::vector<double> lower_bound;
  std::vector<double> upper_bound;
};

struct MPTParam
{
  bool is_hard_fixing_terminal_point;
  int num_curvature_sampling_points;
  double base_point_dist_from_base_link;
  double top_point_dist_from_base_link;
  double mid_point_dist_from_base_link;
  double clearance_from_road;
  double clearance_from_object;
  double base_point_weight;
  double top_point_weight;
  double mid_point_weight;
  double lat_error_weight;
  double yaw_error_weight;
  double steer_input_weight;
  double steer_rate_weight;
  double steer_acc_weight;
  double terminal_lat_error_weight;
  double terminal_yaw_error_weight;
  double terminal_path_lat_error_weight;
  double terminal_path_yaw_error_weight;
  double zero_ff_steer_angle;
};

class MPTOptimizer
{
private:
  bool is_showing_debug_info_;

  std::unique_ptr<osqp::OSQPInterface> osqp_solver_ptr_;
  std::unique_ptr<MPTParam> mpt_param_ptr_;
  std::unique_ptr<TrajectoryParam> traj_param_ptr_;
  std::unique_ptr<QPParam> qp_param_ptr_;
  std::unique_ptr<ConstrainParam> constraint_param_ptr_;
  std::unique_ptr<VehicleParam> vehicle_param_ptr_;
  std::unique_ptr<VehicleModelInterface> vehicle_model_ptr_;

  std::vector<ReferencePoint> convertToReferencePoints(
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
    const geometry_msgs::Pose & ego_pose, const std::unique_ptr<Trajectories> & prev_mpt_points,
    DebugData * debug_data) const;

  std::vector<ReferencePoint> getReferencePoints(
    const geometry_msgs::Pose & origin_pose, const geometry_msgs::Pose & ego_pose,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & points,
    const std::unique_ptr<Trajectories> & prev_mpt_points, DebugData * debug_data) const;

  std::vector<ReferencePoint> getBaseReferencePoints(
    const std::vector<geometry_msgs::Point> & interpolated_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & points) const;

  void calcCurvature(std::vector<ReferencePoint> * ref_points) const;

  void calcArcLength(std::vector<ReferencePoint> * ref_points) const;

  void calcExtraPoints(std::vector<ReferencePoint> * ref_points) const;

  void calcFixPoints(
    const std::unique_ptr<Trajectories> & prev_trajs, const geometry_msgs::Pose & ego_pose,
    std::vector<ReferencePoint> * ref_points, DebugData * debug_data) const;

  /*
 * predict equation: Xec = Aex * x0 + Bex * Uex + Wex
 * cost function: J = Xex' * Qex * Xex + (Uex - Uref)' * R1ex * (Uex - Urefex) + Uex' * R2ex * Uex
 * Qex = diag([Q,Q,...]), R1ex = diag([R,R,...])
 */
  boost::optional<MPTMatrix> generateMPTMatrix(
    const std::vector<ReferencePoint> & reference_points,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points) const;

  void addSteerWeightR(Eigen::MatrixXd * R, const std::vector<ReferencePoint> & ref_points) const;

  void addSteerWeightF(Eigen::VectorXd * f) const;

  boost::optional<Eigen::VectorXd> executeOptimization(
    const bool enable_avoidance, const MPTMatrix & m,
    const std::vector<ReferencePoint> & ref_points,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points, const CVMaps & maps,
    const Eigen::VectorXd & initial_state, DebugData * debug_data);

  std::vector<autoware_planning_msgs::TrajectoryPoint> getMPTPoints(
    const std::vector<ReferencePoint> & ref_points, const Eigen::VectorXd & Uex,
    const MPTMatrix & mpc_matrix, const Eigen::VectorXd & initial_state,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & optimized_points) const;

  double calcLateralError(
    const geometry_msgs::Point & target_point, const ReferencePoint & ref_point) const;

  Eigen::VectorXd getInitialState(
    const geometry_msgs::Pose & ego_pose, const ReferencePoint & nearest_ref_point) const;

  std::vector<Bounds> getReferenceBounds(
    const bool enable_avoidance, const std::vector<ReferencePoint> & ref_points,
    const CVMaps & maps, DebugData * debug_data) const;

  std::vector<double> getBound(
    const bool enable_avoidance, const ReferencePoint & ref_point, const CVMaps & maps) const;

  double getTraversedDistance(
    const bool enable_avoidance, const ReferencePoint & ref_point, const double traverse_angle,
    const double initial_value, const CVMaps & maps,
    const bool search_expanding_side = false) const;

  //TODO: refactor replace all relevant funcs
  double getClearance(
    const cv::Mat & clearance_map, const geometry_msgs::Point & map_point,
    const nav_msgs::MapMetaData & map_info, const double default_dist = 0.0) const;

  ObjectiveMatrix getObjectiveMatrix(const Eigen::VectorXd & x0, const MPTMatrix & m) const;

  ConstraintMatrix getConstraintMatrix(
    const bool enable_avoidance, const Eigen::VectorXd & x0, const MPTMatrix & m,
    const CVMaps & maps, const std::vector<ReferencePoint> & ref_points,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    DebugData * debug_data) const;

public:
  MPTOptimizer(
    const bool is_showing_debug_info, const QPParam & qp_param, const TrajectoryParam & traj_param,
    const ConstrainParam & constrain_param, const VehicleParam & vehicle_param,
    const MPTParam & mpt_param);
  ~MPTOptimizer();

  boost::optional<std::vector<autoware_planning_msgs::TrajectoryPoint>>
  getModelPredictiveTrajectory(
    const bool enable_avoidance,
    const std::vector<autoware_planning_msgs::TrajectoryPoint> & smoothed_points,
    const std::vector<autoware_planning_msgs::PathPoint> & path_points,
    const std::unique_ptr<Trajectories> & prev_trajs, const CVMaps & maps,
    const geometry_msgs::Pose & ego_pose, DebugData * debug_data);
};

#endif

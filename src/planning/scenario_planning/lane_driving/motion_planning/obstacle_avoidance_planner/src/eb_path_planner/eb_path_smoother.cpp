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

#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/MapMetaData.h>
#include <algorithm>
#include <chrono>
#include <memory>
#include <vector>

#include <ros/console.h>
#include <tf2/utils.h>

#include <opencv2/opencv.hpp>

#include <osqp_interface/osqp_interface.h>

#include "eb_path_planner/eb_path_smoother.h"
#include "eb_path_planner/util.h"

EBPathSmoother::EBPathSmoother(double exploring_minimum_radius, double backward_fixing_distance, double fixing_distance,
                               double delta_arc_length_for_path_smoothing, double delta_arc_length_for_explored_points)
    : number_of_sampling_points_(100),
      number_of_diff_optimization_points_for_cold_start_(40),
      exploring_minimum_radius_(exploring_minimum_radius),
      backward_fixing_distance_(backward_fixing_distance),
      fixing_distance_(fixing_distance),
      delta_arc_length_for_path_smoothing_(delta_arc_length_for_path_smoothing),
      delta_arc_length_for_explored_points_(delta_arc_length_for_explored_points),
      loose_constrain_disntance_(0.1) {
  initializeSolver();
}

EBPathSmoother::~EBPathSmoother() {}

void EBPathSmoother::initializeSolver() {
  Eigen::MatrixXd P = makePMatrix();
  std::vector<double> q(number_of_sampling_points_ * 2, 0.0);
  Eigen::MatrixXd A = Eigen::MatrixXd::Identity(number_of_sampling_points_ * 2, number_of_sampling_points_ * 2);
  std::vector<double> lower_bound(number_of_sampling_points_ * 2, 0.0);
  std::vector<double> upper_bound(number_of_sampling_points_ * 2, 0.0);
  const double eps_abs = 1.0e-6;
  osqp_solver_ptr_ = std::make_unique<osqp::OSQPInterface>(P, A, q, lower_bound, upper_bound, eps_abs);
  const double eps_rel = 1.0e-9;
  const int max_iter = 10000;
  osqp_solver_ptr_->updateEpsRel(eps_rel);
  osqp_solver_ptr_->updateMaxIter(max_iter);
}

Eigen::MatrixXd EBPathSmoother::makePMatrix() {
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(number_of_sampling_points_ * 2, number_of_sampling_points_ * 2);
  for (int r = 0; r < number_of_sampling_points_ * 2; r++) {
    for (int c = 0; c < number_of_sampling_points_ * 2; c++) {
      if (r == c) {
        P(r, c) = 6;
      } else if (std::abs(c - r) == 1) {
        P(r, c) = -4;
      } else if (std::abs(c - r) == 2) {
        P(r, c) = 1;
      } else {
        P(r, c) = 0;
      }
    }
  }
  return P;
}

std::vector<autoware_planning_msgs::TrajectoryPoint> EBPathSmoother::generateOptimizedExploredPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<geometry_msgs::Point>& fixed_points, const std::vector<geometry_msgs::Point>& non_fixed_points,
    const geometry_msgs::Pose& ego_pose, const cv::Mat& clearance_map, const nav_msgs::MapMetaData& map_info,
    std::vector<geometry_msgs::Point>& debug_interpolated_points,
    std::vector<geometry_msgs::Point>& debug_constrain_points) {
  std::vector<geometry_msgs::Point> candidate_points;
  for (int i = 0; i < fixed_points.size(); i++) {
    candidate_points.push_back(fixed_points[i]);
  }
  for (int i = 0; i < non_fixed_points.size(); i++) {
    candidate_points.push_back(non_fixed_points[i]);
  }
  if (candidate_points.empty()) {
    ROS_WARN("Need to be checked, empty candidate points");
    std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points;
    for (const auto& point : path_points) {
      autoware_planning_msgs::TrajectoryPoint tmp_point;
      tmp_point.pose = point.pose;
      tmp_point.twist = point.twist;
      traj_points.push_back(tmp_point);
    }
    return traj_points;
  }
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (size_t i = 0; i < candidate_points.size(); i++) {
    tmp_x.push_back(candidate_points[i].x);
    tmp_y.push_back(candidate_points[i].y);
  }
  std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length_for_explored_points_, interpolated_points);
  debug_interpolated_points = interpolated_points;
  // std::cout << "explore fixed size "<< fixed_points.size() << std::endl;
  // std::cout << "explore non fixed size "<< non_fixed_points.size() << std::endl;

  int farrest_idx_from_start_point =
      std::min((int)(number_of_sampling_points_ - 1), (int)(interpolated_points.size() - 1));
  int num_fixed_points = 0;
  if (!fixed_points.empty() && !interpolated_points.empty()) {
    double min_dist = 9999999;
    // int nearest_i = 0;
    for (int i = 0; i < interpolated_points.size(); i++) {
      double dx = interpolated_points[i].x - fixed_points.back().x;
      double dy = interpolated_points[i].y - fixed_points.back().y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist) {
        min_dist = dist;
        num_fixed_points = i;
      }
    }
  }
  num_fixed_points = std::min(num_fixed_points, farrest_idx_from_start_point);
  // std::cout << "farrest idx  "<< farrest_idx_from_start_point << std::endl;
  // std::cout << "num points "<< farrest_idx_from_start_point << std::endl;
  Mode qp_mode = Mode::Avoidance;
  updateQPConstrain(interpolated_points, farrest_idx_from_start_point, num_fixed_points, clearance_map, map_info,
                    qp_mode, debug_constrain_points);

  std::vector<double> optval = solveQP();

  std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points;
  for (size_t i = 0; i <= farrest_idx_from_start_point; i++) {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = optval[i];
    tmp_point.pose.position.y = optval[i + number_of_sampling_points_];
    tmp_point.pose.position.z = 0;
    traj_points.push_back(tmp_point);
  }
  for (int i = 0; i < traj_points.size(); i++) {
    double yaw = 0;
    if (i == traj_points.size() - 1) {
      double dx = traj_points[i].pose.position.x - traj_points[i - 1].pose.position.x;
      double dy = traj_points[i].pose.position.y - traj_points[i - 1].pose.position.y;
      yaw = std::atan2(dy, dx);
    } else {
      double dx = traj_points[i + 1].pose.position.x - traj_points[i].pose.position.x;
      double dy = traj_points[i + 1].pose.position.y - traj_points[i].pose.position.y;
      yaw = std::atan2(dy, dx);
    }
    tf2::Quaternion quaternion;
    double roll = 0;
    double pitch = 0;
    quaternion.setRPY(roll, pitch, yaw);
    traj_points[i].pose.orientation = tf2::toMsg(quaternion);
  }
  return traj_points;
}

std::vector<autoware_planning_msgs::TrajectoryPoint> EBPathSmoother::generateOptimizedPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<geometry_msgs::Pose>& fixed_points, const std::vector<geometry_msgs::Pose>& non_fixed_points,
    std::vector<geometry_msgs::Point>& debug_interpolated, std::vector<geometry_msgs::Point>& debug_constrain) {
  // TODO: consider interface aware implementatpn pose or point?
  std::vector<geometry_msgs::Point> candidate_points;
  for (int i = 0; i < fixed_points.size(); i++) {
    candidate_points.push_back(fixed_points[i].position);
  }
  for (int i = 0; i < non_fixed_points.size(); i++) {
    candidate_points.push_back(non_fixed_points[i].position);
  }
  if (candidate_points.empty()) {
    ROS_WARN("Need to be checked, empty candidate points");
    std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points;
    for (const auto& point : path_points) {
      autoware_planning_msgs::TrajectoryPoint tmp_point;
      tmp_point.pose = point.pose;
      tmp_point.twist = point.twist;
      traj_points.push_back(tmp_point);
    }
    return traj_points;
  }
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (size_t i = 0; i < candidate_points.size(); i++) {
    tmp_x.push_back(candidate_points[i].x);
    tmp_y.push_back(candidate_points[i].y);
  }
  std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length_for_path_smoothing_, interpolated_points);
  debug_interpolated = interpolated_points;

  Mode qp_mode = Mode::LaneFollowing;
  // interpolate point enough to have at least number of sampling points
  int farrest_idx_from_start_point =
      std::min((int)(number_of_sampling_points_ - 1), (int)(interpolated_points.size() - 1));
  // int num_fixed_points = std::max((int)fixed_points.size()- 3 -1, 0);
  // num_fixed_points = std::min(num_fixed_points, farrest_idx_from_start_point);
  int num_fixed_points = 0;
  if (!fixed_points.empty() && !interpolated_points.empty()) {
    double min_dist = 999999999;
    // int nearest_i = 0;
    for (int i = 0; i < interpolated_points.size(); i++) {
      double dx = interpolated_points[i].x - fixed_points.back().position.x;
      double dy = interpolated_points[i].y - fixed_points.back().position.y;
      double dist = std::sqrt(dx * dx + dy * dy);
      if (dist < min_dist) {
        min_dist = dist;
        num_fixed_points = i;
      }
    }
  }
  num_fixed_points = std::min(num_fixed_points, farrest_idx_from_start_point);

  // std::cout << "interpolated size "<< interpolated_points.size() << std::endl;
  // std::cout << "fixed points size "<< fixed_points.size() << std::endl;
  // std::cout << "farrest idx  "<< farrest_idx_from_start_point << std::endl;
  // std::cout << "path fixed size "<< fixed_points.size() << std::endl;
  // std::cout << "path non fixed size "<< non_fixed_points.size() << std::endl;
  // std::cout << "num fixed points " << num_fixed_points << std::endl;
  // std::cout << "farrest idx  "<< farrest_idx_from_start_point << std::endl;
  updateQPConstrain(interpolated_points, num_fixed_points, farrest_idx_from_start_point, debug_constrain);
  std::vector<double> optval = solveQP();

  std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points;
  for (size_t i = 0; i <= farrest_idx_from_start_point; i++) {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = optval[i];
    tmp_point.pose.position.y = optval[i + number_of_sampling_points_];
    tmp_point.pose.position.z = 0;
    traj_points.push_back(tmp_point);
  }
  for (int i = 0; i < traj_points.size(); i++) {
    double yaw = 0;
    if (i == traj_points.size() - 1) {
      double dx = traj_points[i].pose.position.x - traj_points[i - 1].pose.position.x;
      double dy = traj_points[i].pose.position.y - traj_points[i - 1].pose.position.y;
      yaw = std::atan2(dy, dx);
    } else {
      double dx = traj_points[i + 1].pose.position.x - traj_points[i].pose.position.x;
      double dy = traj_points[i + 1].pose.position.y - traj_points[i].pose.position.y;
      yaw = std::atan2(dy, dx);
    }
    tf2::Quaternion quaternion;
    double roll = 0;
    double pitch = 0;
    quaternion.setRPY(roll, pitch, yaw);
    traj_points[i].pose.orientation = tf2::toMsg(quaternion);
  }
  return traj_points;
}

std::vector<double> EBPathSmoother::solveQP() {
  std::chrono::high_resolution_clock::time_point begin = std::chrono::high_resolution_clock::now();
  std::tuple<std::vector<double>, std::vector<double>, int> result = osqp_solver_ptr_->optimize();
  std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  // std::cout << "taken iter " << osqp_solver_ptr_->getTakenIter() << std::endl;
  // std::cout << "opt time " << time.count() / (1000.0 * 1000.0) << " ms" << std::endl;
  return std::get<0>(result);
}

void EBPathSmoother::updateQPConstrain(const std::vector<geometry_msgs::Point>& interpolated_points,
                                       const int farrest_point_idx, const int num_fixed_points,
                                       const cv::Mat& clearance_map, const nav_msgs::MapMetaData& map_info,
                                       const Mode qp_optimization_mode,
                                       std::vector<geometry_msgs::Point>& debug_constrain_points) {
  std::vector<double> lower_bound(number_of_sampling_points_ * 2, 0.0);
  std::vector<double> upper_bound(number_of_sampling_points_ * 2, 0.0);

  for (int i = 0; i < number_of_sampling_points_; ++i) {
    if (i == 0) {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else if (i == 1)  // second initial x
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else if (i == farrest_point_idx - 1)  // second last x
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else if (i == farrest_point_idx)  // last x
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else if (i > farrest_point_idx) {
      lower_bound[i] = interpolated_points[farrest_point_idx].x;
      upper_bound[i] = interpolated_points[farrest_point_idx].x;
    } else if (i < num_fixed_points) {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else {
      if (qp_optimization_mode == Mode::Avoidance) {
        double min_constrain_buffer = loose_constrain_disntance_;
        if (i >= num_fixed_points - 2 && i <= num_fixed_points + 10) {
          min_constrain_buffer = 0.6;
        }
        geometry_msgs::Point interpolated_p_in_image;
        float clearance;
        if (util::transformMapToImage(interpolated_points[i], map_info, interpolated_p_in_image)) {
          clearance = clearance_map.ptr<float>((int)interpolated_p_in_image.y)[(int)interpolated_p_in_image.x] *
                      map_info.resolution;
        } else {
          clearance = 0.25;
        }
        float diff = std::fmax(clearance - exploring_minimum_radius_, min_constrain_buffer);
        lower_bound[i] = interpolated_points[i].x - diff;
        upper_bound[i] = interpolated_points[i].x + diff;
      } else {
        lower_bound[i] = interpolated_points[i].x - 0.2;
        upper_bound[i] = interpolated_points[i].x + 0.2;
      }
    }
  }
  for (int i = 0; i < number_of_sampling_points_; ++i) {
    if (i == 0)  // initial x
    {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else if (i == 1)  // second initial x
    {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else if (i == farrest_point_idx - 1)  // second last x
    {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else if (i == farrest_point_idx)  // last x
    {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else if (i >= farrest_point_idx) {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[farrest_point_idx].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[farrest_point_idx].y;
    } else if (i < num_fixed_points) {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else {
      if (qp_optimization_mode == Mode::Avoidance) {
        double min_constrain_buffer = loose_constrain_disntance_;
        if (i >= num_fixed_points - 2 && i <= num_fixed_points + 10) {
          min_constrain_buffer = 0.6;
        }
        geometry_msgs::Point interpolated_p = interpolated_points[i];
        geometry_msgs::Point interpolated_p_in_image;
        float clearance;
        if (util::transformMapToImage(interpolated_points[i], map_info, interpolated_p_in_image)) {
          clearance = clearance_map.ptr<float>((int)interpolated_p_in_image.y)[(int)interpolated_p_in_image.x] *
                      map_info.resolution;
        } else {
          clearance = 0.5;
        }
        float diff = std::fmax(clearance - exploring_minimum_radius_, min_constrain_buffer);
        lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y - diff;
        upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y + diff;

        interpolated_p.z = diff;
        debug_constrain_points.push_back(interpolated_p);
      } else {
        lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y - 0.2;
        upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y + 0.2;
      }
    }
  }
  osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
}

void EBPathSmoother::updateQPConstrain(const std::vector<geometry_msgs::Point>& interpolated_points,
                                       const int num_fixed_points, const int farrest_point_idx,
                                       std::vector<geometry_msgs::Point>& debug_constrain_points) {
  std::vector<double> lower_bound(number_of_sampling_points_ * 2, 0.0);
  std::vector<double> upper_bound(number_of_sampling_points_ * 2, 0.0);

  for (int i = 0; i < number_of_sampling_points_; ++i) {
    if (i == 0) {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else if (i == 1)  // second initial x
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else if (i == farrest_point_idx - 1)  // second last x
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else if (i == farrest_point_idx)  // last x
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else if (i > farrest_point_idx) {
      lower_bound[i] = interpolated_points[farrest_point_idx].x;
      upper_bound[i] = interpolated_points[farrest_point_idx].x;
    } else if (i < num_fixed_points) {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    } else if (i >= num_fixed_points && i <= num_fixed_points + 2) {
      lower_bound[i] = interpolated_points[i].x - 1.0;
      upper_bound[i] = interpolated_points[i].x + 1.0;
      geometry_msgs::Point tmp_p = interpolated_points[i];
      tmp_p.z = 1.0;
      debug_constrain_points.push_back(tmp_p);
    } else {
      lower_bound[i] = interpolated_points[i].x - 0.2;
      upper_bound[i] = interpolated_points[i].x + 0.2;
      geometry_msgs::Point tmp_p = interpolated_points[i];
      tmp_p.z = 0.2;
      debug_constrain_points.push_back(tmp_p);
    }
  }
  for (int i = 0; i < number_of_sampling_points_; ++i) {
    if (i == 0)  // initial x
    {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else if (i == 1)  // second initial x
    {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else if (i == farrest_point_idx - 1)  // second last x
    {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else if (i == farrest_point_idx)  // last x
    {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else if (i > farrest_point_idx) {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[farrest_point_idx].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[farrest_point_idx].y;
    } else if (i < num_fixed_points) {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y;
    } else if (i >= num_fixed_points && i <= num_fixed_points + 2) {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y - 1.0;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y + 1.0;
    } else {
      lower_bound[i + number_of_sampling_points_] = interpolated_points[i].y - 0.2;
      upper_bound[i + number_of_sampling_points_] = interpolated_points[i].y + 0.2;
    }
  }
  osqp_solver_ptr_->updateBounds(lower_bound, upper_bound);
}
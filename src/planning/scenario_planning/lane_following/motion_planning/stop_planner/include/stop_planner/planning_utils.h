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

#pragma once

#define EIGEN_MPL2_ONLY
#include <tf/transform_datatypes.h>
#include <tf2/utils.h>
#include <tf2_eigen/tf2_eigen.h>
#include <Eigen/Core>
#include <Eigen/Geometry>

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>

// User defined includes
#include "autoware_planning_msgs/Trajectory.h"

// C++ includes
#include <memory>

namespace planning_utils {

constexpr double ERROR = 1e-6;

double calcDistance2D(const geometry_msgs::Point& p, const geometry_msgs::Point& q);
double calcDistSquared2D(const geometry_msgs::Point& p, const geometry_msgs::Point& q);
double calcLateralError2D(const geometry_msgs::Point& a_start, const geometry_msgs::Point& a_end,
                          const geometry_msgs::Point& b);
bool findClosestIdxWithDistAngThr(const autoware_planning_msgs::Trajectory& in_trajectory,
                                  const geometry_msgs::Pose& curr_pose, int32_t& out_idx, double dist_thr = 3.0,
                                  double angle_thr = M_PI_2);  // TODO: more test

// refer from apache's pointinpoly in http://www.visibone.com/inpoly/
template <typename T>
bool isInPolygon(const std::vector<T>& polygon, const T& point);
template <>
bool isInPolygon(const std::vector<geometry_msgs::Point>& polygon, const geometry_msgs::Point& point);

geometry_msgs::Point transformToAbsoluteCoordinate2D(const geometry_msgs::Point& point,
                                                     const geometry_msgs::Pose& current_pose);
geometry_msgs::Point transformToAbsoluteCoordinate3D(const geometry_msgs::Point& point,
                                                     const geometry_msgs::Pose& origin);  // TODO: test
geometry_msgs::Point transformToRelativeCoordinate2D(const geometry_msgs::Point& point,
                                                     const geometry_msgs::Pose& current_pose);
geometry_msgs::Point transformToRelativeCoordinate3D(const geometry_msgs::Point& point,
                                                     const geometry_msgs::Pose& current_pose);  // TODO: test
geometry_msgs::Quaternion getQuaternionFromYaw(const double& _yaw);

double normalizeEulerAngle(double euler);
void convertEulerAngleToMonotonic(std::vector<double>& a);
bool linearInterpTrajectory(const std::vector<double>& base_index,
                            const autoware_planning_msgs::Trajectory& base_trajectory,
                            const std::vector<double>& out_index, autoware_planning_msgs::Trajectory& out_trajectory);
int calcForwardIdxByLineIntegral(const autoware_planning_msgs::Trajectory& in_trajectory, int32_t start_idx,
                                 double distance);

geometry_msgs::Pose getPoseOnTrajectoryWithRadius(const autoware_planning_msgs::Trajectory& in_tajectory,
                                                  const geometry_msgs::Point& origin, const int start_idx,
                                                  const double distance);

}  // namespace planning_utils

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

namespace planning_utils
{

constexpr double ERROR = 1e-6;

double calcDistance2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q);
double calcDistSquared2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q);
double calcLateralError2D(const geometry_msgs::Point &a_start, const geometry_msgs::Point &a_end,
                          const geometry_msgs::Point &b);
std::vector<geometry_msgs::Pose> extractPoses(const autoware_planning_msgs::Trajectory &lane);
std::pair<bool, int32_t> findClosestIdxWithDistAngThr(const std::vector<geometry_msgs::Pose> &curr_ps,
                                                      const geometry_msgs::Pose &curr_pose,
                                                      double dist_thr = 3.0,
                                                      double angle_thr = M_PI_2); // TODO: more test

// 0 : front, 1 : reverse, 2 :invalid
int8_t getLaneDirection(const std::vector<geometry_msgs::Pose> &poses, double dist_thr = 0.05);
bool isDirectionForward(const geometry_msgs::Pose &prev, const geometry_msgs::Pose &next);
bool isDirectionForward(const geometry_msgs::Pose &prev, const geometry_msgs::Point &next);

// refer from apache's pointinpoly in http://www.visibone.com/inpoly/
template <typename T>
bool isInPolygon(const std::vector<T> &polygon, const T &point);
template <>
bool isInPolygon(const std::vector<geometry_msgs::Point> &polygon, const geometry_msgs::Point &point);

double kmph2mps(double velocity_kmph);
double normalizeEulerAngle(double euler);
geometry_msgs::Point transformToAbsoluteCoordinate2D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &current_pose);
geometry_msgs::Point transformToAbsoluteCoordinate3D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &origin); // TODO: test
geometry_msgs::Point transformToRelativeCoordinate2D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &current_pose);
geometry_msgs::Point transformToRelativeCoordinate3D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &current_pose); // TODO: test
geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw);

std::pair<bool, geometry_msgs::Point> calcFootOfPerpendicular(const geometry_msgs::Point &line_s, const geometry_msgs::Point &line_e, const geometry_msgs::Point &point);

// which_point: 0 = forward, 1 = backward
std::pair<bool, geometry_msgs::Point> findIntersectionWithLineCircle(const geometry_msgs::Point &line_s, const geometry_msgs::Point &line_e, const geometry_msgs::Point &point, double range, int8_t which_point = 0);

// which_dir: 0 = forward, 1 = backward
std::pair<bool, int32_t> findFirstIdxOutOfRange(const std::vector<geometry_msgs::Pose> &pose_v,
                                                const geometry_msgs::Point &point,
                                                double range, int32_t search_idx_s, int8_t which_dir);

// which dir 0 : forward 1 : backward
std::tuple<bool, int32_t, geometry_msgs::Pose> calcDistanceConsideredPoseAndIdx(const autoware_planning_msgs::Trajectory &lane, const geometry_msgs::Pose &pose,
                                                                                int32_t idx, double stop_dist, int8_t which_dir);


}  // namespace planning_utils

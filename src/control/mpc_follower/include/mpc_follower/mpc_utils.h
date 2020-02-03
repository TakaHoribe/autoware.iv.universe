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

#pragma once

#include <cmath>
#include <vector>
#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include "autoware_planning_msgs/Trajectory.h"
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "mpc_follower/interpolate.h"

#include "mpc_follower/mpc_trajectory.h"

namespace MPCUtils
{

/**
 * @brief convert from yaw to ros-Quaternion
 * @param [in] yaw input yaw angle
 * @return quaternion
 */
geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw);

/**
 * @brief normalize angle into [-pi to pi]
 * @param [in] _angle input angle
 * @return normalized angle
 */
double normalizeRadian(const double _angle);

/**
 * @brief convert eular angle vector including +-2pi to 0 jump to continuous series data 
 * @param [out] a input angle vector
 */
void convertEulerAngleToMonotonic(std::vector<double> &a);

double calcDist2d(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1);
double calcDist3d(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1);
double calcSquaredDist2d(const geometry_msgs::Point &p0, const geometry_msgs::Point &p1);

void convertToMPCTrajectory(const autoware_planning_msgs::Trajectory &input, MPCTrajectory &output);
void calcMPCTrajectoryArclength(const MPCTrajectory &trajectory, std::vector<double> &arclength);
bool resampleMPCTrajectorySpline(const MPCTrajectory &input, const double resample_interval_dist, MPCTrajectory &output);
bool linearInterpMPCTrajectory(const std::vector<double> &in_index, const MPCTrajectory &in_traj,
                               const std::vector<double> &out_index, MPCTrajectory &out_traj);
bool splineInterpMPCTrajectory(const std::vector<double> &in_index, const MPCTrajectory &in_traj,
                               const std::vector<double> &out_index, MPCTrajectory &out_traj);
void calcMPCTrajectoryTime(MPCTrajectory &traj);

/**
 * @brief calculate yaw angle in MPCTrajectory from xy vector
 * @param [inout] traj object trajectory
 */
void calcTrajectoryYawFromXY(MPCTrajectory &traj);

/**
 * @brief Calculate path curvature by 3-points circle fitting with smoothing num (use nearest 3 points when num = 1)
 * @param [inout] traj object trajectory
 * @param [in] curvature_smoothing_num index distance for 3 points for curvature calculation
 */
void calcTrajectoryCurvature(MPCTrajectory &traj, int curvature_smoothing_num);

/**
 * @brief calculate nearest pose on MPCTrajectory
 * @param [in] traj reference trajectory
 * @param [in] self_pose object pose 
 * @param [out] nearest_pose nearest pose on path
 * @param [out] nearest_index path index of nearest pose 
 * @param [out] min_dist_error distance error from nearest pose to self pose
 * @param [out] nearest_yaw_error yaw angle error from nearest pose to self pose
 * @param [out] nearest_time time of nearest pose on trajectory
 * @return false when nearest pose couldn't find for some reasons
 */
bool calcNearestPose(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                     unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);

/**
 * @brief calculate nearest pose on MPCTrajectory with linear interpolation
 * @param [in] traj reference trajectory
 * @param [in] self_pose object pose 
 * @param [out] nearest_pose nearest pose on path
 * @param [out] nearest_index path index of nearest pose 
 * @param [out] min_dist_error distance error from nearest pose to self pose
 * @param [out] nearest_yaw_error yaw angle error from nearest pose to self pose
 * @param [out] nearest_time time of nearest pose on trajectory
 * @return false when nearest pose couldn't find for some reasons
 */
bool calcNearestPoseInterp(const MPCTrajectory &traj, const geometry_msgs::Pose &self_pose, geometry_msgs::Pose &nearest_pose,
                           unsigned int &nearest_index, double &min_dist_error, double &nearest_yaw_error, double &nearest_time);

/**
 * @brief convert MPCTraj to visualizaton marker for visualization
 */
void convertTrajToMarker(const MPCTrajectory &traj, visualization_msgs::MarkerArray &markers,
                         std::string ns, double r, double g, double b, double z, std::string &frame_id);

}; // namespace MPCUtils
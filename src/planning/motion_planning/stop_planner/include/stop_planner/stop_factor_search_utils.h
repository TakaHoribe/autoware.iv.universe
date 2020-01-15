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

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>

// User defined includes
#include "autoware_planning_msgs/Trajectory.h"
#include "stop_planner/planning_utils.h"


// C++ includes
#include <chrono>
#include <memory>

namespace motion_planner
{
    
using PolygonX = std::vector<geometry_msgs::Point>;
typedef std::pair<int32_t, int8_t> StopFactorPair; // idx, kind


// display the next waypoint by markers.
visualization_msgs::Marker displayWall(const geometry_msgs::Pose &pose, int8_t kind, int32_t id = 0);
std::vector<geometry_msgs::Point> createLattice(const geometry_msgs::Pose &pose, double height, double width,
                                                double count);

std::unique_ptr<std_msgs::ColorRGBA> setColorDependsOnObstacleKind(int8_t kind);
std_msgs::ColorRGBA setColorWhite();
std_msgs::ColorRGBA setColorGray();
std_msgs::ColorRGBA setColorYellow();

std::pair<bool, int32_t> calcForwardIdxByLineIntegral(const autoware_planning_msgs::Trajectory &lane, int32_t base_idx, double stop_offset_dist);

bool findPointCloudInPolygon(const PolygonX &poly, const sensor_msgs::PointCloud2 &pc, const int32_t points_thr, std::vector<geometry_msgs::Point> &out_pcd);
bool findPointsInPolygon(const PolygonX &poly, const std::vector<geometry_msgs::Point> &points, const int32_t points_thr, std::vector<geometry_msgs::Point> &out_points);


geometry_msgs::Point calcClosestPointByXAxis(const geometry_msgs::Pose &pose, const std::vector<geometry_msgs::Point> &points);

}  // namespace motion_planner

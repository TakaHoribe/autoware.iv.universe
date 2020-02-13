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
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// User defined includes
#include "libplanning_utils/planning_utils.h"

// C++ includes
#include <memory>

namespace waypoint_follower {
const auto MAP_FRAME = "map";

// visualizer
// display the next waypoint by markers.
std::unique_ptr<visualization_msgs::Marker> displayNextWaypoint(const geometry_msgs::Point& position);
// display the next target by markers.
std::unique_ptr<visualization_msgs::Marker> displayNextTarget(const geometry_msgs::Point& target);

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> generateTrajectoryCircle(const geometry_msgs::Point& target,
                                                           const geometry_msgs::Pose& current_pose);
// display the locus of pure pursuit by markers.
std::unique_ptr<visualization_msgs::Marker> displayTrajectoryCircle(
    const std::vector<geometry_msgs::Point>& traj_circle_array);

// display the search radius by markers.
std::unique_ptr<visualization_msgs::Marker> displaySearchRadius(const geometry_msgs::Point& current_pose,
                                                                double search_radius);

visualization_msgs::MarkerArray displayControlTrajectory(const autoware_planner_msgs::Trajectory& traj);

}  // namespace waypoint_follower

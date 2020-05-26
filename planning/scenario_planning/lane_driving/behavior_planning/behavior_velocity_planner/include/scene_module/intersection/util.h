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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

namespace util
{
bool setVelocityFrom(
  const size_t idx, const double vel, autoware_planning_msgs::PathWithLaneId * input);

bool splineInterpolate(
  const autoware_planning_msgs::PathWithLaneId & input, const double interval,
  autoware_planning_msgs::PathWithLaneId * output);

int insertPoint(
  const geometry_msgs::Pose & in_pose, autoware_planning_msgs::PathWithLaneId * inout_path);

geometry_msgs::Pose getAheadPose(
  const size_t start_idx, const double ahead_dist,
  const autoware_planning_msgs::PathWithLaneId & path);

bool isAheadOf(const geometry_msgs::Pose & target, const geometry_msgs::Pose & origin);
bool hasLaneId(const autoware_planning_msgs::PathPointWithLaneId & p, const int id);
}  // namespace util

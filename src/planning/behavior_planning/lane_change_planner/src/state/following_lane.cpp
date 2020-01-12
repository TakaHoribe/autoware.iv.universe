/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <lane_change_planner/state/following_lane.h>
#include <lane_change_planner/data_manager.h>
#include <lane_change_planner/route_handler.h>
#include <lane_change_planner/utilities.h>

namespace lane_change_planner
{
State FollowingLaneState::getCurrentState() const
{
  return State::FOLLOWING_LANE;
}

void FollowingLaneState::entry()
{
}

autoware_planning_msgs::PathWithLaneId FollowingLaneState::getPath() const
{
  return status_.lane_follow_path;
}

void FollowingLaneState::update()
{
  // update input data
  {
    if (!SingletonDataManager::getInstance().getCurrentSelfVelocity(current_twist_))
    {
      ROS_ERROR_STREAM("Failed to get self velocity. Using previous velocity.");
    }
    if (!SingletonDataManager::getInstance().getCurrentSelfPose(current_pose_))
    {
      ROS_ERROR_STREAM("Failed to get self pose. Using previous pose.");
    }
    if (!SingletonDataManager::getInstance().getDynamicObjects(dynamic_objects_))
    {
      ROS_ERROR_STREAM("Failed to get dynamic objects. Using previous objects.");
    }
  }

  // update path
  {
    double backward_path_length = 5;
    double forward_path_length = 100;
    status_.lane_follow_path = RouteHandler::getInstance().getReferencePath(current_pose_.pose, backward_path_length, forward_path_length);

    if (!RouteHandler::getInstance().isInPreferredLane(current_pose_) && current_twist_ != nullptr)
    {
      status_.lane_change_path = RouteHandler::getInstance().getLaneChangePath(current_pose_.pose, current_twist_->twist);
    }
  }
}

State FollowingLaneState::getNextState() const
{
  if (RouteHandler::getInstance().isInPreferredLane(current_pose_))
  {
    return State::FOLLOWING_LANE;
  }
  if (isTooCloseToDeadEnd())
  {
    return State::FORCING_LANE_CHANGE;
  }
  if (isLaneChangeable())
  {
    return State::EXECUTING_LANE_CHANGE;
  }
  return State::FOLLOWING_LANE;
}

bool FollowingLaneState::isVehicleInPreferredLane() const
{
  return RouteHandler::getInstance().isInPreferredLane(current_pose_);
}

bool FollowingLaneState::isTooCloseToDeadEnd() const
{
  return false;
}

bool FollowingLaneState::isLaneChangeable() const
{
  const auto& target_lanelets = RouteHandler::getInstance().getLaneChangeTarget(current_pose_.pose);
  if (target_lanelets.empty())
  {
    return false;
  }
  if (dynamic_objects_ == nullptr)
  {
    return true;
  }
  auto object_indices = util::filterObjectsByLanelets(*dynamic_objects_, target_lanelets);

  const double min_thresh = 5;
  const double stop_time = 2.0;
  const double buffer = 2;
  const double time_resolution = 0.5;
  const auto& vehicle_predicted_path =
      util::convertToPredictedPath(status_.lane_change_path, current_twist_->twist, current_pose_.pose);

  for (const auto& i : object_indices)
  {
    const auto& obj = dynamic_objects_->objects.at(i);
    for (const auto& obj_path : obj.state.predicted_paths)
    {
      double distance = util::getDistanceBetweenPredictedPaths(obj_path, vehicle_predicted_path, time_resolution, 2.5);
      double thresh = util::l2Norm(obj.state.twist_covariance.twist.linear) * stop_time;
      thresh = (thresh > min_thresh) ? thresh : min_thresh;
      thresh += buffer;
      if (distance < thresh)
      {
        return false;
      }
    }
  }
  return true;
}

}  // namespace lane_change_planner

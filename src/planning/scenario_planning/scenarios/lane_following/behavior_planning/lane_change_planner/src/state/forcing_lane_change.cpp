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

#include <lane_change_planner/state/forcing_lane_change.h>
#include <lane_change_planner/data_manager.h>
#include <lane_change_planner/route_handler.h>
#include <lane_change_planner/utilities.h>

namespace lane_change_planner
{
State ForcingLaneChangeState::getCurrentState() const
{
  return State::FORCING_LANE_CHANGE;
}

void ForcingLaneChangeState::entry(const Status& status)
{
  status_ = status;
  if (!SingletonDataManager::getInstance().getLaneChangerParameters(ros_parameters_))
  {
    ROS_ERROR_STREAM("Failed to get parameters. Please check if you set ROS parameters correctly.");
  }
  original_lanes_ = RouteHandler::getInstance().getLaneletsFromIds(status_.lane_follow_lane_ids);
  target_lanes_ = RouteHandler::getInstance().getLaneletsFromIds(status_.lane_change_lane_ids);
}

autoware_planning_msgs::PathWithLaneId ForcingLaneChangeState::getPath() const
{
  return status_.lane_change_path;
}

void ForcingLaneChangeState::update()
{
  if (!SingletonDataManager::getInstance().getCurrentSelfPose(current_pose_))
  {
    ROS_ERROR("failed to get current pose");
  }

  // update path
  {

    lanelet::ConstLanelet closest_original_lane;
    lanelet::ConstLanelet closest_target_lane;
    if (!lanelet::utils::query::getClosestLanelet(original_lanes_, current_pose_.pose, &closest_original_lane))
    {
      return;
    }
    if (!lanelet::utils::query::getClosestLanelet(target_lanes_, current_pose_.pose, &closest_target_lane))
    {
      return;
    }

    double backward_length = ros_parameters_.backward_path_length;
    double forward_length = ros_parameters_.forward_path_length;
    const auto trimmed_original_lanes = RouteHandler::getInstance().getLaneletSequence(
        closest_original_lane, current_pose_.pose, backward_length, forward_length);
    const auto trimmed_target_lanes = RouteHandler::getInstance().getLaneletSequence(
        closest_target_lane, current_pose_.pose, backward_length, forward_length);

    lanelet::ConstLanelets lanes;
    lanes.insert(lanes.end(), trimmed_original_lanes.begin(), trimmed_original_lanes.end());
    lanes.insert(lanes.end(), trimmed_target_lanes.begin(), trimmed_target_lanes.end());

    const double width = ros_parameters_.drivable_area_width;
    const double height = ros_parameters_.drivable_area_height;
    const double resolution = ros_parameters_.drivable_area_resolution;
    status_.lane_change_path.drivable_area =
        util::convertLanesToDrivableArea(lanes, current_pose_, width, height, resolution);
  }
}

State ForcingLaneChangeState::getNextState() const
{
  if (hasFinishedLaneChange())
  {
    return State::FOLLOWING_LANE;
  }
  return State::FORCING_LANE_CHANGE;
}

bool ForcingLaneChangeState::hasFinishedLaneChange() const
{
  static ros::Time start_time = ros::Time::now();

  if (RouteHandler::getInstance().isInTargetLane(current_pose_, target_lanes_))
  {
    return (ros::Time::now() - start_time > ros::Duration(2));
  }
  else
  {
    start_time = ros::Time::now();
  }
  return false;
}

}  // namespace lane_change_planner

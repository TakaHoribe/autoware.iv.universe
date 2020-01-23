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

#include <lane_change_planner/state/executing_lane_change.h>
#include <lane_change_planner/data_manager.h>
#include <lane_change_planner/route_handler.h>
#include <lane_change_planner/utilities.h>

namespace lane_change_planner
{
State ExecutingLaneChangeState::getCurrentState() const
{
  return State::EXECUTING_LANE_CHANGE;
}

void ExecutingLaneChangeState::entry(const Status& status)
{
  status_ = status;
  if (!SingletonDataManager::getInstance().getLaneChangerParameters(ros_parameters_))
  {
    ROS_ERROR_STREAM("Failed to get parameters. Please check if you set ROS parameters correctly.");
  }
  original_lanes_ = RouteHandler::getInstance().getLaneletsFromIds(status_.lane_follow_lane_ids);
  target_lanes_ = RouteHandler::getInstance().getLaneletsFromIds(status_.lane_change_lane_ids);
}

autoware_planning_msgs::PathWithLaneId ExecutingLaneChangeState::getPath() const
{
  return status_.lane_change_path;
}

void ExecutingLaneChangeState::update()
{
  if (!SingletonDataManager::getInstance().getCurrentSelfPose(current_pose_))
  {
    ROS_ERROR("failed to get current pose");
  }
  if (!SingletonDataManager::getInstance().getCurrentSelfVelocity(current_twist_))
  {
    ROS_ERROR_STREAM("Failed to get self velocity. Using previous velocity");
  }
  if (!SingletonDataManager::getInstance().getDynamicObjects(dynamic_objects_))
  {
    ROS_ERROR_STREAM("Failed to get dynamic objects. Using previous objects");
  }

  // update path
  {
    const double width = ros_parameters_.drivable_area_width;
    const double height = ros_parameters_.drivable_area_height;
    const double resolution = ros_parameters_.drivable_area_resolution;
    lanelet::ConstLanelets lanes;
    lanes.insert(lanes.end(), original_lanes_.begin(), original_lanes_.end());
    lanes.insert(lanes.end(), target_lanes_.begin(), target_lanes_.end());
    status_.lane_change_path.drivable_area =
        util::convertLanesToDrivableArea(lanes, current_pose_, width, height, resolution);
  }
}

State ExecutingLaneChangeState::getNextState() const
{
  if (isStillOnOriginalLane() && !isTargetLaneStillClear())
  {
    return State::FOLLOWING_LANE;
  }

  if (hasFinishedLaneChange())
  {
    return State::FOLLOWING_LANE;
  }
  return State::EXECUTING_LANE_CHANGE;
}

bool ExecutingLaneChangeState::isStillOnOriginalLane() const
{
  lanelet::BasicPoint2d vehicle_pose2d(current_pose_.pose.position.x, current_pose_.pose.position.y);
  for (const auto& llt : original_lanes_)
  {
    double distance = lanelet::geometry::distance2d(llt.polygon2d().basicPolygon(), vehicle_pose2d);
    if (distance < std::numeric_limits<double>::epsilon())
    {
      return true;
    }
  }
  return false;
}

bool ExecutingLaneChangeState::isTargetLaneStillClear() const
{
  const double min_thresh = ros_parameters_.min_stop_distance;
  const double stop_time = ros_parameters_.stop_time;
  const double time_resolution = ros_parameters_.prediction_time_resolution;
  const double prediction_duration = ros_parameters_.prediction_duration;

  if (target_lanes_.empty())
  {
    return false;
  }
  auto object_indices = util::filterObjectsByLanelets(*dynamic_objects_, target_lanes_);
  const auto& vehicle_predicted_path =
      util::convertToPredictedPath(status_.lane_change_path, current_twist_->twist, current_pose_.pose);

  for (const auto& i : object_indices)
  {
    const auto& obj = dynamic_objects_->objects.at(i);
    for (const auto& obj_path : obj.state.predicted_paths)
    {
      double distance = util::getDistanceBetweenPredictedPaths(obj_path, vehicle_predicted_path, time_resolution,
                                                               prediction_duration);
      double thresh = util::l2Norm(obj.state.twist_covariance.twist.linear) * stop_time;
      thresh = std::max(thresh, min_thresh);
      if (distance < thresh)
      {
        return false;
      }
    }
  }
  return true;
}

bool ExecutingLaneChangeState::hasFinishedLaneChange() const
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

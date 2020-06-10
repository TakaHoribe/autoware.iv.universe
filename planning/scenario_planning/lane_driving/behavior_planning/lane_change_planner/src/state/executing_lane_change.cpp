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

#include <lane_change_planner/data_manager.h>
#include <lane_change_planner/route_handler.h>
#include <lane_change_planner/state/common_functions.h>
#include <lane_change_planner/state/executing_lane_change.h>
#include <lane_change_planner/utilities.h>

#include <lanelet2_extension/utility/utilities.h>
#include <tf2/utils.h>

namespace lane_change_planner
{
ExecutingLaneChangeState::ExecutingLaneChangeState(
  const Status & status, const std::shared_ptr<DataManager> & data_manager_ptr,
  const std::shared_ptr<RouteHandler> & route_handler_ptr)
: StateBase(status, data_manager_ptr, route_handler_ptr)
{
}

State ExecutingLaneChangeState::getCurrentState() const { return State::EXECUTING_LANE_CHANGE; }

void ExecutingLaneChangeState::entry()
{
  ros_parameters_ = data_manager_ptr_->getLaneChangerParameters();

  original_lanes_ = route_handler_ptr_->getLaneletsFromIds(status_.lane_follow_lane_ids);
  target_lanes_ = route_handler_ptr_->getLaneletsFromIds(status_.lane_change_lane_ids);
  status_.lane_change_available = false;
  status_.lane_change_ready = false;
}

autoware_planning_msgs::PathWithLaneId ExecutingLaneChangeState::getPath() const
{
  return status_.lane_change_path;
}

void ExecutingLaneChangeState::update()
{
  current_twist_ = data_manager_ptr_->getCurrentSelfVelocity();
  current_pose_ = data_manager_ptr_->getCurrentSelfPose();
  dynamic_objects_ = data_manager_ptr_->getDynamicObjects();

  // update path
  {
    lanelet::ConstLanelets lanes;
    lanes.insert(lanes.end(), original_lanes_.begin(), original_lanes_.end());
    lanes.insert(lanes.end(), target_lanes_.begin(), target_lanes_.end());

    const double width = ros_parameters_.drivable_area_width;
    const double height = ros_parameters_.drivable_area_height;
    const double resolution = ros_parameters_.drivable_area_resolution;
    status_.lane_change_path.drivable_area =
      util::convertLanesToDrivableArea(lanes, current_pose_, width, height, resolution);
  }
}

State ExecutingLaneChangeState::getNextState() const
{
  if (isAbortConditionSatisfied()) {
    return State::FOLLOWING_LANE;
  }

  if (hasFinishedLaneChange()) {
    return State::FOLLOWING_LANE;
  }
  return State::EXECUTING_LANE_CHANGE;
}

bool ExecutingLaneChangeState::isAbortConditionSatisfied() const
{
  // check abort enable flag
  if (!ros_parameters_.enable_abort_lane_change) {
    return false;
  }

  // check if lane change path is still safe
  bool is_path_safe = state_machine::common_functions::isLaneChangePathSafe(
    status_.lane_change_path, original_lanes_, target_lanes_, dynamic_objects_, current_pose_.pose,
    current_twist_->twist, ros_parameters_, false);

  // check vehicle velocity thresh
  bool is_velocity_low =
    util::l2Norm(current_twist_->twist.linear) < ros_parameters_.abort_lane_change_velocity_thresh;

  // find closest lanelet in original lane
  lanelet::ConstLanelet closest_lanelet;
  lanelet::BasicPoint2d vehicle_pose2d(
    current_pose_.pose.position.x, current_pose_.pose.position.y);
  double min_distance = std::numeric_limits<double>::max();
  for (const auto & llt : original_lanes_) {
    double distance = lanelet::geometry::distance2d(llt.polygon2d().basicPolygon(), vehicle_pose2d);
    if (distance < min_distance) {
      closest_lanelet = llt;
      min_distance = distance;
    }
  }

  // check if vehicle is within lane
  const auto lane_length = lanelet::utils::getLaneletLength2d(original_lanes_);
  const auto lane_poly = lanelet::utils::getPolygonFromArcLength(original_lanes_, 0, lane_length);
  const auto vehicle_poly = util::getVehiclePolygon(
    current_pose_.pose, ros_parameters_.vehicle_width, ros_parameters_.base_link2front);
  bool is_within_original_lane = boost::geometry::within(
    lanelet::utils::to2D(vehicle_poly).basicPolygon(),
    lanelet::utils::to2D(lane_poly).basicPolygon());

  // check distance from original lane
  const auto centerline2d = lanelet::utils::to2D(closest_lanelet.centerline()).basicLineString();
  double distance = lanelet::geometry::distance2d(centerline2d, vehicle_pose2d);
  bool is_distance_small = distance < ros_parameters_.abort_lane_change_distance_thresh;

  // check angle thresh from original lane
  const double lane_angle =
    lanelet::utils::getLaneletAngle(closest_lanelet, current_pose_.pose.position);
  const double vehicle_yaw = tf2::getYaw(current_pose_.pose.orientation);
  const double yaw_diff = util::normalizeRadian(lane_angle - vehicle_yaw);
  bool is_angle_diff_small = std::abs(yaw_diff) < ros_parameters_.abort_lane_change_angle_thresh;

  // abort only if velocity is low or vehicle pose is close enough
  if (!is_path_safe) {
    if (is_velocity_low && is_within_original_lane) {
      return true;
    }
    if (is_distance_small && is_angle_diff_small) {
      return true;
    }
    ROS_WARN_STREAM_THROTTLE(
      1,
      "DANGER!!! New danger was detected during lane change, but it is too late to abort! "
      "Please "
      "be catious");
  }

  return false;
}  // namespace lane_change_planner

bool ExecutingLaneChangeState::hasFinishedLaneChange() const
{
  static ros::Time start_time = ros::Time::now();

  if (route_handler_ptr_->isInTargetLane(current_pose_, target_lanes_)) {
    return (ros::Time::now() - start_time > ros::Duration(2));
  } else {
    start_time = ros::Time::now();
  }
  return false;
}

}  // namespace lane_change_planner

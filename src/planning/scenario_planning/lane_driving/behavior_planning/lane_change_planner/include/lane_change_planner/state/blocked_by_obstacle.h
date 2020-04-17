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

#ifndef LANE_CHANGE_PLANNER_STATE_BLOCKED_BY_OBSTACLE_H
#define LANE_CHANGE_PLANNER_STATE_BLOCKED_BY_OBSTACLE_H

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <lane_change_planner/state/state_base_class.h>
#include <lanelet2_core/primitives/Primitive.h>
#include <memory>

namespace lane_change_planner {
class BlockedByObstacleState : public StateBase {
 private:
  geometry_msgs::PoseStamped current_pose_;
  std::shared_ptr<geometry_msgs::TwistStamped const> current_twist_;
  std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> dynamic_objects_;
  bool lane_change_approved_;
  bool force_lane_change_;
  bool found_safe_path_;
  lanelet::ConstLanelets current_lanes_;
  lanelet::ConstLanelets lane_change_lanes_;

  // State transition conditions
  bool foundSafeLaneChangePath() const;
  bool hasEnoughDistance() const;
  bool isLaneChangeApproved() const;
  bool isLaneBlocked() const;
  bool isOutOfCurrentLanes() const;
  bool isLaneChangeAvailable() const;
  bool isLaneChangeReady() const;
  bool laneChangeForcedByOperator() const;
  bool isLaneChangePathSafe(const lanelet::ConstLanelets& target_lanes,
                            const autoware_planning_msgs::PathWithLaneId& path) const;

 public:
  BlockedByObstacleState(const Status& status, const std::shared_ptr<DataManager>& data_manager_ptr,
             const std::shared_ptr<RouteHandler>& route_handler_ptr);

  // override virtual functions
  void entry() override;
  void update() override;
  State getNextState() const override;
  State getCurrentState() const override;
  autoware_planning_msgs::PathWithLaneId getPath() const override;
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_STATE_BLOCKED_BY_OBSTACLE_H

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

#ifndef LANE_CHANGE_PLANNER_STATE_FOLLOWING_LANE_H
#define LANE_CHANGE_PLANNER_STATE_FOLLOWING_LANE_H

#include <lane_change_planner/state/state_base_class.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>

#include <memory>

namespace lane_change_planner
{
class FollowingLaneState : public StateBase
{
private:
  geometry_msgs::PoseStamped current_pose_;
  std::shared_ptr<geometry_msgs::TwistStamped const> current_twist_;
  std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> dynamic_objects_;
  autoware_planning_msgs::PathWithLaneId lane_change_path_;

  // State transition conditions
  bool isVehicleInPreferredLane() const;
  bool isTooCloseToDeadEnd() const;
  bool isLaneChangeable() const;

public:
  FollowingLaneState() = default;

  // override virtual functions
  void entry() override;
  void update() override;
  State getNextState() const override;
  State getCurrentState() const override;
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_STATE_FOLLOWING_LANE_H
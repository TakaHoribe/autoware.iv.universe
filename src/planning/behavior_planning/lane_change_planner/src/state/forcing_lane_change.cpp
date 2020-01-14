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
namespace lane_change_planner
{
State ForcingLaneChangeState::getCurrentState() const
{
  return State::FORCING_LANE_CHANGE;
}

void ForcingLaneChangeState::entry()
{
  // locked_path = getReferenceLine(current_pose, original_lane);
}

autoware_planning_msgs::PathWithLaneId ForcingLaneChangeState::getPath() const
{
  return status_.lane_change_path;
}

void ForcingLaneChangeState::update()
{
  // return locked_path;
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
  return false;
}

}  // namespace lane_change_planner

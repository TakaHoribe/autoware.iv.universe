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

#ifndef LANE_CHANGE_PLANNER_STATE_STATE_BASE_CLASS_H
#define LANE_CHANGE_PLANNER_STATE_STATE_BASE_CLASS_H

#include <autoware_planning_msgs/PathWithLaneId.h>
#include <string>
#include <iostream>

namespace lane_change_planner
{
enum State
{
  NO_STATE,
  FOLLOWING_LANE,
  EXECUTING_LANE_CHANGE,
  ABORTING_LANE_CHANGE,
  FORCING_LANE_CHANGE,
};

std::ostream& operator<<(std::ostream& ostream, const State& state);

class StateBase
{
protected:
  autoware_planning_msgs::PathWithLaneId path_;

public:
  virtual void entry() = 0;
  virtual void update() = 0;
  virtual State getNextState() const = 0;
  virtual State getCurrentState() const = 0;

  autoware_planning_msgs::PathWithLaneId getPath() const;
};
}  // namespace lane_change_planner

#endif  // LANE_CHANGE_PLANNER_STATE_STATE_BASE_CLASS_H
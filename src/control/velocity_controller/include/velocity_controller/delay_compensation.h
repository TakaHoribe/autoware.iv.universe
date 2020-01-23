/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#ifndef VELOCITY_CONTROLLER_DELAY_COMPENSATION
#define VELOCITY_CONTROLLER_DELAY_COMPENSATION

#include <autoware_planning_msgs/Trajectory.h>
#include "velocity_controller_mathutils.h"

class DelayCompensator
{
public:
  DelayCompensator();
  ~DelayCompensator();

  static double computeCommandAccelerationAfterDelayTime(
      const autoware_planning_msgs::Trajectory& trajectory, int32_t closest_waypoint_index,
      double delay_time, double current_velocity)
  {
    const double delay_distance = current_velocity * delay_time;

    double sum_distance = 0.0;
    for (unsigned int i = closest_waypoint_index; i < trajectory.points.size() - 1; ++i)
    {
      sum_distance += vcutils::calcDistance2D(trajectory.points.at(i).pose, trajectory.points.at(i + 1).pose);
      if (sum_distance > delay_distance)
      {
        return trajectory.points.at(i).accel.linear.x;
      }
    }

    return trajectory.points.back().accel.linear.x;
  }
};

#endif

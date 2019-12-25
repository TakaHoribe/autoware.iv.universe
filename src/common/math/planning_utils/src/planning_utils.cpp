/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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
 *
 * Author: Robin Karlsson
 */

#include "planning_utils/planning_utils.h"

namespace planning_utils
{

double normalizeEulerAngle(double euler)
{
  double res = euler;
  while (res > M_PI)
  {
    res -= (2.0 * M_PI);
  }
  while (res < -M_PI)
  {
    res += 2.0 * M_PI;
  }

  return res;
}

geometry_msgs::Quaternion getQuaternionFromYaw(double yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

template<class T> 
bool calcClosestIndex(const T &path, const geometry_msgs::Pose &pose, int &closest, double dist_thr, double angle_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  double yaw_pose = tf2::getYaw(pose.orientation);
  closest = -1;

  for (int i = 0; i < (int)path.points.size(); ++i)
  {
    const double dist_squared = calcSquaredDist2d(getPose(path, i), pose);

    /* check distance threshold */
    if (dist_squared > dist_thr * dist_thr)
      continue;

    /* check angle threshold */
    double yaw_i = tf2::getYaw(getPose(path, i).orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_i);

    if (std::fabs(yaw_diff) > angle_thr)
      continue;

    if (dist_squared < dist_squared_min)
    {
      dist_squared_min = dist_squared;
      closest = i;
    }
  }

  return closest == -1 ? false : true;
}

template bool calcClosestIndex<autoware_planning_msgs::Trajectory>(const autoware_planning_msgs::Trajectory &path, const geometry_msgs::Pose &pose,
                                                                   int &closest, double dist_thr, double angle_thr);
template bool calcClosestIndex<autoware_planning_msgs::PathWithLaneId>(const autoware_planning_msgs::PathWithLaneId &path, const geometry_msgs::Pose &pose,
                                                                       int &closest, double dist_thr, double angle_thr);
template bool calcClosestIndex<autoware_planning_msgs::Path>(const autoware_planning_msgs::Path &path, const geometry_msgs::Pose &pose,
                                                             int &closest, double dist_thr, double angle_thr);

} // namespace planning_utils

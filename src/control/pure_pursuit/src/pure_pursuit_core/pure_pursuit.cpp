/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
 */

#include "pure_pursuit/pure_pursuit.h"
#include "pure_pursuit/util/planning_utils.h"

namespace planning_utils {

bool PurePursuit::isDataReady() {
  if (!curr_wps_ptr_) return false;
  if (!curr_pose_ptr_) return false;
  return true;
}

std::pair<bool, double> PurePursuit::run() {
  if (!isDataReady()) return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());

  auto clst_pair = findClosestIdxWithDistAngThr(*curr_wps_ptr_, *curr_pose_ptr_, clst_thr_dist_, clst_thr_ang_);

  if (!clst_pair.first) {
    ROS_WARN("cannot find, curr_bool: %d, clst_idx: %d", clst_pair.first, clst_pair.second);
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  int32_t next_wp_idx = findNextPointIdx(clst_pair.second);
  if (next_wp_idx == -1) {
    ROS_WARN("lost next waypoint");
    return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
  }

  loc_next_wp_ = curr_wps_ptr_->at(next_wp_idx).position;

  geometry_msgs::Point next_tgt_pos;
  // if next waypoint is first
  if (next_wp_idx == 0) {
    next_tgt_pos = curr_wps_ptr_->at(next_wp_idx).position;
  } else {
    // linear interpolation
    std::pair<bool, geometry_msgs::Point> lerp_pair = lerpNextTarget(next_wp_idx);

    if (!lerp_pair.first) {
      ROS_WARN("lost target! ");
      return std::make_pair(false, std::numeric_limits<double>::quiet_NaN());
    }

    next_tgt_pos = lerp_pair.second;
  }
  loc_next_tgt_ = next_tgt_pos;

  double kappa = calcCurvature(next_tgt_pos, *curr_pose_ptr_);

  return std::make_pair(true, kappa);
}

// linear interpolation of next target
std::pair<bool, geometry_msgs::Point> PurePursuit::lerpNextTarget(int32_t next_wp_idx) {
  constexpr double ERROR2 = 1e-5;  // 0.00001
  const geometry_msgs::Point& vec_end = curr_wps_ptr_->at(next_wp_idx).position;
  const geometry_msgs::Point& vec_start = curr_wps_ptr_->at(next_wp_idx - 1).position;
  const geometry_msgs::Pose& curr_pose = *curr_pose_ptr_;

  Eigen::Vector3d vec_a((vec_end.x - vec_start.x), (vec_end.y - vec_start.y), (vec_end.z - vec_start.z));

  if (vec_a.norm() < ERROR2) {
    ROS_ERROR("waypoint interval is almost 0");
    return std::make_pair(false, geometry_msgs::Point());
  }

  const double lateral_error = calcLateralError2D(vec_start, vec_end, curr_pose.position);

  if (fabs(lateral_error) > lookahead_distance_) {
    ROS_ERROR("lateral error is larger than lookahead distance");
    ROS_ERROR("lateral error: %lf, lookahead distance: %lf", lateral_error, lookahead_distance_);
    return std::make_pair(false, geometry_msgs::Point());
  }

  /* calculate the position of the foot of a perpendicular line */
  Eigen::Vector2d uva2d(vec_a.x(), vec_a.y());
  uva2d.normalize();
  Eigen::Rotation2Dd rot = (lateral_error > 0) ? Eigen::Rotation2Dd(-M_PI / 2.0) : Eigen::Rotation2Dd(M_PI / 2.0);
  Eigen::Vector2d uva2d_rot = rot * uva2d;

  geometry_msgs::Point h;
  h.x = curr_pose.position.x + fabs(lateral_error) * uva2d_rot.x();
  h.y = curr_pose.position.y + fabs(lateral_error) * uva2d_rot.y();
  h.z = curr_pose.position.z;

  // if there is a intersection
  if (fabs(fabs(lateral_error) - lookahead_distance_) < ERROR2) {
    return std::make_pair(true, h);
  } else {
    // if there are two intersection
    // get intersection in front of vehicle
    const double s = sqrt(pow(lookahead_distance_, 2) - pow(lateral_error, 2));
    geometry_msgs::Point res;
    res.x = h.x + s * uva2d.x();
    res.y = h.y + s * uva2d.y();
    res.z = curr_pose.position.z;
    return std::make_pair(true, res);
  }
}

int32_t PurePursuit::findNextPointIdx(int32_t search_start_idx) {
  // if waypoints are not given, do nothing.
  if (curr_wps_ptr_->empty() || search_start_idx == -1) return -1;

  // look for the next waypoint.
  for (int32_t i = search_start_idx; i < (int32_t)curr_wps_ptr_->size(); i++) {
    // if search waypoint is the last
    if (i == ((int32_t)curr_wps_ptr_->size() - 1)) {
      return i;
    }

    // if waypoint direction is forward
    const auto gld = planning_utils::getLaneDirection(*curr_wps_ptr_, 0.05);
    if (gld == 0) {
      // if waypoint is not in front of ego, skip
      auto ret = planning_utils::transformToRelativeCoordinate2D(curr_wps_ptr_->at(i).position, *curr_pose_ptr_);
      if (ret.x < 0) {
        continue;
      }
    } else if (gld == 1) {
      // waypoint direction is backward

      // if waypoint is in front of ego, skip
      auto ret = planning_utils::transformToRelativeCoordinate2D(curr_wps_ptr_->at(i).position, *curr_pose_ptr_);
      if (ret.x > 0) {
        continue;
      }
    } else {
      return -1;
    }

    const geometry_msgs::Point& curr_motion_point = curr_wps_ptr_->at(i).position;
    const geometry_msgs::Point& curr_pose_point = curr_pose_ptr_->position;
    // if there exists an effective waypoint
    const double ds = calcDistSquared2D(curr_motion_point, curr_pose_point);
    if (ds > std::pow(lookahead_distance_, 2)) return i;
  }

  // if this program reaches here , it means we lost the waypoint!
  return -1;
}

void PurePursuit::setCurrentPose(const geometry_msgs::Pose& msg) {
  curr_pose_ptr_ = std::make_shared<geometry_msgs::Pose>();
  *curr_pose_ptr_ = msg;
}

void PurePursuit::setWaypoints(const std::vector<geometry_msgs::Pose>& msg) {
  curr_wps_ptr_ = std::make_shared<std::vector<geometry_msgs::Pose>>();
  *curr_wps_ptr_ = msg;
}

}  // namespace planning_utils

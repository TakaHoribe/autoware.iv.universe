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
#include "ros/ros.h"

#include "utilization/interpolate.h"
#include "utilization/util.h"
#include "scene_module/intersection/util.h"

// clang-format off
#define DEBUG_INFO(...) { ROS_INFO_COND(show_debug_info_, __VA_ARGS__); }

// clang-format on
namespace util
{
int insertPoint(
  const geometry_msgs::Pose & in_pose, autoware_planning_msgs::PathWithLaneId * inout_path)
{
  static constexpr double dist_thr = 10.0;
  static constexpr double angle_thr = M_PI / 1.5;
  int closest = -1;
  if (!planning_utils::calcClosestIndex(*inout_path, in_pose, closest, dist_thr, angle_thr)) {
    return -1;
  }
  int insert_idx = closest;
  if (isAheadOf(in_pose, inout_path->points.at(closest).point.pose)) {
    ++insert_idx;
  }

  autoware_planning_msgs::PathPointWithLaneId inserted_point;
  inserted_point = inout_path->points.at(closest);
  inserted_point.point.pose = in_pose;

  auto it = inout_path->points.begin() + insert_idx;
  inout_path->points.insert(it, inserted_point);
  return insert_idx;
}

bool splineInterpolate(
  const autoware_planning_msgs::PathWithLaneId & input, const double interval,
  autoware_planning_msgs::PathWithLaneId * output)
{
  *output = input;

  static constexpr double ep = 1.0e-8;

  // calc arclength for path
  std::vector<double> base_x;
  std::vector<double> base_y;
  for (const auto & p : input.points) {
    base_x.push_back(p.point.pose.position.x);
    base_y.push_back(p.point.pose.position.y);
  }
  std::vector<double> base_s = interpolation::calcEuclidDist(base_x, base_y);
  std::vector<double> resampled_s;
  for (double d = 0.0; d < base_s.back() - ep; d += interval) {
    resampled_s.push_back(d);
  }

  // do spline for xy
  interpolation::SplineInterpolate spline;
  std::vector<double> resampled_x;
  std::vector<double> resampled_y;
  if (
    !spline.interpolate(base_s, base_x, resampled_s, resampled_x) ||
    !spline.interpolate(base_s, base_y, resampled_s, resampled_y)) {
    ROS_ERROR("[IntersectionModule::splineInterpolate] spline interpolation failed.");
    return false;
  }

  // set xy
  output->points.clear();
  for (size_t i = 0; i < resampled_s.size(); i++) {
    autoware_planning_msgs::PathPointWithLaneId p;
    p.point.pose.position.x = resampled_x.at(i);
    p.point.pose.position.y = resampled_y.at(i);
    output->points.push_back(p);
  }

  // set yaw
  for (int i = 1; i < static_cast<int>(resampled_s.size()) - 1; i++) {
    auto p = output->points.at(i - 1).point.pose.position;
    auto n = output->points.at(i + 1).point.pose.position;
    double yaw = std::atan2(n.y - p.y, n.x - p.x);
    output->points.at(i).point.pose.orientation = planning_utils::getQuaternionFromYaw(yaw);
  }
  if (output->points.size() > 1) {
    size_t l = resampled_s.size();
    output->points.front().point.pose.orientation = output->points.at(1).point.pose.orientation;
    output->points.back().point.pose.orientation = output->points.at(l - 1).point.pose.orientation;
  }
  return true;
}

geometry_msgs::Pose getAheadPose(
  const size_t start_idx, const double ahead_dist,
  const autoware_planning_msgs::PathWithLaneId & path)
{
  if (path.points.size() == 0) {
    return geometry_msgs::Pose{};
  }

  double curr_dist = 0.0;
  double prev_dist = 0.0;
  for (size_t i = start_idx; i < path.points.size() - 1 && i >= 0; ++i) {
    const geometry_msgs::Pose p0 = path.points.at(i).point.pose;
    const geometry_msgs::Pose p1 = path.points.at(i + 1).point.pose;
    curr_dist += planning_utils::calcDist2d(p0, p1);
    if (curr_dist > ahead_dist) {
      const double dl = std::max(curr_dist - prev_dist, 0.0001 /* avoid 0 divide */);
      const double w_p0 = (curr_dist - ahead_dist) / dl;
      const double w_p1 = (ahead_dist - prev_dist) / dl;
      geometry_msgs::Pose p;
      p.position.x = w_p0 * p0.position.x + w_p1 * p1.position.x;
      p.position.y = w_p0 * p0.position.y + w_p1 * p1.position.y;
      p.position.z = w_p0 * p0.position.z + w_p1 * p1.position.z;
      tf2::Quaternion q0_tf, q1_tf;
      tf2::fromMsg(p0.orientation, q0_tf);
      tf2::fromMsg(p1.orientation, q1_tf);
      p.orientation = tf2::toMsg(q0_tf.slerp(q1_tf, w_p1));
      return p;
    }
    prev_dist = curr_dist;
  }
  return path.points.back().point.pose;
}

bool setVelocityFrom(
  const size_t idx, const double vel, autoware_planning_msgs::PathWithLaneId * input)
{
  for (size_t i = idx; i < input->points.size(); ++i) {
    input->points.at(i).point.twist.linear.x =
      std::min(vel, input->points.at(i).point.twist.linear.x);
  }
  return true;
}

bool isAheadOf(const geometry_msgs::Pose & target, const geometry_msgs::Pose & origin)
{
  geometry_msgs::Pose p = planning_utils::transformRelCoordinate2D(target, origin);
  bool is_target_ahead = (p.position.x > 0.0);
  return is_target_ahead;
}

bool hasLaneId(const autoware_planning_msgs::PathPointWithLaneId & p, const int id)
{
  for (const auto & pid : p.lane_ids) {
    if (pid == id) return true;
  }
  return false;
}

}  // namespace util
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
#include <scene_module/crosswalk/scene_walkway.h>

#include <cmath>

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;
using Line = bg::model::linestring<Point>;

WalkwayModule::WalkwayModule(const int64_t module_id, const lanelet::ConstLanelet & walkway)
: SceneModuleInterface(module_id), walkway_(walkway), state_(State::APPROACH)
{
}

bool WalkwayModule::modifyPathVelocity(autoware_planning_msgs::PathWithLaneId * path)
{
  debug_data_ = {};
  debug_data_.base_link2front = planner_data_->base_link2front;
  first_stop_path_point_index_ = static_cast<int>(path->points.size()) - 1;

  const auto input = *path;

  if (state_ == State::APPROACH) {
    // create polygon
    lanelet::CompoundPolygon3d lanelet_polygon = walkway_.polygon3d();
    Polygon polygon;
    for (const auto & lanelet_point : lanelet_polygon) {
      polygon.outer().push_back(bg::make<Point>(lanelet_point.x(), lanelet_point.y()));
    }
    polygon.outer().push_back(polygon.outer().front());

    autoware_planning_msgs::PathWithLaneId output;
    if (!insertTargetVelocityPoint(input, polygon, stop_margin_, 0.0, output)) return false;

    // update state
    const Point self_pose = {planner_data_->current_pose.pose.position.x,
                             planner_data_->current_pose.pose.position.y};
    const double distance = bg::distance(polygon, self_pose);
    const double distance_threshold = stop_margin_ + planner_data_->base_link2front + 1.0;
    if (distance < distance_threshold && planner_data_->isVehicleStopping()) state_ = State::STOP;
    return true;
  } else if (state_ == State::STOP) {
    if (planner_data_->isVehicleStopping()) {
      state_ = State::SURPASSED;
    }
    return true;
  } else if (state_ == State::SURPASSED) {
    return true;
  }
}

bool WalkwayModule::insertTargetVelocityPoint(
  const autoware_planning_msgs::PathWithLaneId & input,
  const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &
    polygon,
  const double & margin, const double & velocity, autoware_planning_msgs::PathWithLaneId & output)
{
  output = input;
  for (size_t i = 0; i < output.points.size() - 1; ++i) {
    Line line = {
      {output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y},
      {output.points.at(i + 1).point.pose.position.x,
       output.points.at(i + 1).point.pose.position.y}};
    std::vector<Point> collision_points;
    bg::intersection(polygon, line, collision_points);

    if (collision_points.empty()) continue;
    // -- debug code --
    for (const auto & collision_point : collision_points) {
      Eigen::Vector3d point3d;
      point3d << collision_point.x(), collision_point.y(),
        planner_data_->current_pose.pose.position.z;
      debug_data_.collision_points.push_back(point3d);
    }
    std::vector<Eigen::Vector3d> line3d;
    Eigen::Vector3d point3d;
    point3d << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y,
      output.points.at(i).point.pose.position.z;
    line3d.push_back(point3d);
    point3d << output.points.at(i + 1).point.pose.position.x,
      output.points.at(i + 1).point.pose.position.y, output.points.at(i + 1).point.pose.position.z;
    line3d.push_back(point3d);
    debug_data_.collision_lines.push_back(line3d);
    // ----------------

    // check nearest collision point
    Point nearest_collision_point;
    double min_dist;
    for (size_t j = 0; j < collision_points.size(); ++j) {
      double dist = bg::distance(
        Point(output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y),
        collision_points.at(j));
      if (j == 0 || dist < min_dist) {
        min_dist = dist;
        nearest_collision_point = collision_points.at(j);
      }
    }

    // search target point index
    size_t insert_target_point_idx = 0;
    const double base_link2front = planner_data_->base_link2front;
    double length_sum = 0;

    const double target_length = margin + base_link2front;
    Eigen::Vector2d point1, point2;
    point1 << nearest_collision_point.x(), nearest_collision_point.y();
    point2 << output.points.at(i).point.pose.position.x, output.points.at(i).point.pose.position.y;
    length_sum += (point2 - point1).norm();
    for (size_t j = i; 0 < j; --j) {
      if (target_length < length_sum) {
        insert_target_point_idx = j + 1;
        break;
      }
      point1 << output.points.at(j).point.pose.position.x,
        output.points.at(j).point.pose.position.y;
      point2 << output.points.at(j - 1).point.pose.position.x,
        output.points.at(j - 1).point.pose.position.y;
      length_sum += (point2 - point1).norm();
    }

    // create target point
    Eigen::Vector2d target_point;
    autoware_planning_msgs::PathPointWithLaneId target_point_with_lane_id;
    getBackwordPointFromBasePoint(point2, point1, point2, length_sum - target_length, target_point);
    const int target_velocity_point_idx =
      std::max(static_cast<int>(insert_target_point_idx - 1), 0);
    target_point_with_lane_id = output.points.at(target_velocity_point_idx);
    target_point_with_lane_id.point.pose.position.x = target_point.x();
    target_point_with_lane_id.point.pose.position.y = target_point.y();
    target_point_with_lane_id.point.twist.linear.x = velocity;
    if (velocity == 0.0 && target_velocity_point_idx < first_stop_path_point_index_) {
      first_stop_path_point_index_ = target_velocity_point_idx;
    }
    // -- debug code --
    if (velocity == 0.0)
      debug_data_.stop_poses.push_back(target_point_with_lane_id.point.pose);
    else
      debug_data_.slow_poses.push_back(target_point_with_lane_id.point.pose);
    // ----------------

    // insert target point
    output.points.insert(
      output.points.begin() + insert_target_point_idx, target_point_with_lane_id);

    // insert 0 velocity after target point
    for (size_t j = insert_target_point_idx; j < output.points.size(); ++j)
      output.points.at(j).point.twist.linear.x =
        std::min(velocity, output.points.at(j).point.twist.linear.x);
    return true;
  }
  return false;
}

bool WalkwayModule::getBackwordPointFromBasePoint(
  const Eigen::Vector2d & line_point1, const Eigen::Vector2d & line_point2,
  const Eigen::Vector2d & base_point, const double backward_length, Eigen::Vector2d & output_point)
{
  Eigen::Vector2d line_vec = line_point2 - line_point1;
  Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
  output_point = base_point + backward_vec;
  return true;
}

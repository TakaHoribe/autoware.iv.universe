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
#include <scene_module/intersection/scene.h>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_core/primitives/BasicRegulatoryElements.h>
#include <lanelet2_extension/utility/utilities.h>

#include "utilization/boost_geometry_helper.h"
#include "utilization/util.h"
#include "utilization/interpolate.h"
#include "scene_module/intersection/util.h"

// clang-format off
#define DEBUG_INFO(...) { ROS_INFO_COND(show_debug_info_, __VA_ARGS__); }

// clang-format on
namespace bg = boost::geometry;

IntersectionModule::IntersectionModule(const int64_t module_id, const int64_t lane_id)
: SceneModuleInterface(module_id), lane_id_(lane_id)
{
  state_machine_.setMarginTime(2.0);  // [sec]
  show_debug_info_ = true;
  approaching_speed_to_stopline_ = 100000.0;
  path_expand_width_ = 2.0;
}

bool IntersectionModule::modifyPathVelocity(autoware_planning_msgs::PathWithLaneId * path)
{
  debug_data_ = {};

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  DEBUG_INFO("[IntersectionModule::run] assigned_lane_id = %ld", lane_id_);
  DEBUG_INFO("[IntersectionModule::run] current_state = %d", static_cast<int>(current_state));

  /* get current pose */
  geometry_msgs::PoseStamped current_pose = planner_data_->current_pose;

  /* check if the current_pose is ahead from judgement line */
  int closest = -1;
  if (!planning_utils::calcClosestIndex(input_path, current_pose.pose, closest)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModule::run] calcClosestIndex fail");
    return false;
  }

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* get detection area */
  std::vector<lanelet::ConstLanelet> objective_lanelets;
  getObjectiveLanelets(lanelet_map_ptr, routing_graph_ptr, lane_id_, &objective_lanelets);
  debug_data_.intersection_detection_lanelets = objective_lanelets;
  DEBUG_INFO(
    "[IntersectionModuleManager::run] lane_id = %ld, objective_lanelets.size = %lu", lane_id_,
    objective_lanelets.size());
  if (objective_lanelets.empty()) {
    DEBUG_INFO("[IntersectionModule::run]: no detection area. skip computation.");
    return true;
  }

ROS_WARN("before path.size = %lu", path->points.size());
  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int judge_line_idx = -1;
  if (!generateStopLine(closest, objective_lanelets, path, &stop_line_idx, &judge_line_idx)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModule::run] setStopLineIdx fail");
    return false;
  }
ROS_WARN(
  "after path.size = %lu, stop_line_idx = %d, judge_line_idx = %d", path->points.size(),
  stop_line_idx, judge_line_idx);

if (stop_line_idx <= 0 || judge_line_idx <= 0) {
  DEBUG_INFO(
    "[IntersectionModule::run] the stop line or judge line is at path[0], ignore "
    "planning. Maybe it is far behind the current position.");
  return true;
  }

  if (current_state == State::STOP) {
    // visualize virtual_wall at vehicle front position
    debug_data_.virtual_wall_pose =
      util::getAheadPose(stop_line_idx, planner_data_->base_link2front, *path);
  }
ROS_WARN(
  "after path.size = %lu, stop_line_idx = %d, judge_line_idx = %d", path->points.size(),
  stop_line_idx, judge_line_idx);
  debug_data_.stop_point_pose = path->points.at(stop_line_idx).point.pose;
  debug_data_.judge_point_pose = path->points.at(judge_line_idx).point.pose;
  debug_data_.path_with_judgeline = *path;

  /* set approaching speed to stop-line */
  util::setVelocityFrom(judge_line_idx, approaching_speed_to_stopline_, path);

  geometry_msgs::Pose judge_line = path->points.at(judge_line_idx).point.pose;
  if (current_state == State::GO && util::isAheadOf(current_pose.pose, judge_line)) {
    DEBUG_INFO("[IntersectionModule::run] over the judge line. no plan needed.");
    return true;  // no plan needed.
  }

  /* get dynamic object */
  const auto objects_ptr = planner_data_->dynamic_objects;

  /* calculate dynamic collision around detection area */
  if (checkCollision(*path, objective_lanelets, objects_ptr, path_expand_width_)) {
    state_machine_.setStateWithMarginTime(State::STOP);
  } else {
    state_machine_.setStateWithMarginTime(State::GO);
  }

  /* set stop speed */
  if (state_machine_.getState() == State::STOP) {
    constexpr double stop_vel = 0.0;
    util::setVelocityFrom(stop_line_idx, stop_vel, path);
  }

  return true;
}

bool IntersectionModule::generateStopLine(
  const int closest, const std::vector<lanelet::ConstLanelet> objective_lanelets,
  autoware_planning_msgs::PathWithLaneId * path, int * stop_line_idx, int * judge_line_idx) const
{
  /* set judge line dist */
  const double current_vel = planner_data_->current_velocity->twist.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold_;
  const double judge_line_dist = planning_utils::calcJudgeLineDist(current_vel, max_acc, 0.0);

  /* set parameters */
  constexpr double interval = 0.2;
  const double mergin_dist = 1.0 + planner_data_->base_link2front;
  const int mergin_idx_dist = std::ceil(mergin_dist / interval);
  const int judge_idx_dist = std::ceil(judge_line_dist / interval);

  /* spline interpolation */
  autoware_planning_msgs::PathWithLaneId path_ip;
  if (!util::splineInterpolate(*path, interval, &path_ip)) return false;
  debug_data_.spline_path = path_ip;

  // find the first path point inside the objective_lanelets
  int first_idx_inside_lanelet = -1;
  ROS_INFO(
    "path_ip.size = %lu, objective_lanelets.size = %lu", path_ip.points.size(),
    objective_lanelets.size());
  for (size_t i = 0; i < path_ip.points.size(); ++i) {
    bool is_in_lanelet = false;
    auto p = path_ip.points.at(i).point.pose.position;
    int j = 0;
    for (const auto & lane : objective_lanelets) {
      is_in_lanelet = bg::within(to_bg2d(p), lane.polygon2d());
      if (is_in_lanelet) {
        ROS_WARN("i = %lu, j = %d, is_in_lanelet = %d", i, j, is_in_lanelet);
        first_idx_inside_lanelet = static_cast<int>(i);
        break;
      }
      ++j;
    }
    if (is_in_lanelet) break;
  }
  if (first_idx_inside_lanelet == -1)
  {
    ROS_INFO("no stop line exist.");
    return false;
  }
  const int stop_idx_ip = std::max(first_idx_inside_lanelet - mergin_idx_dist, 0);
  const int judge_idx_ip = std::max(stop_idx_ip - judge_idx_dist, 0);
  ROS_WARN(
    "stop_idx_ip = %d, judge_idx_ip = %d, mergin_dist = %f, mergin_idx_dist = %d", stop_idx_ip,
    judge_idx_ip, mergin_dist, mergin_idx_dist);

  /* insert stop_line & judge_line (judge line must be inserted at first) */
  const auto inserted_judge_point = path_ip.points.at(judge_idx_ip).point.pose;
  *judge_line_idx = util::insertPoint(inserted_judge_point, path);
  if (stop_idx_ip == judge_idx_ip) {
    *stop_line_idx = *judge_line_idx;
  } else {
    const auto inserted_stop_point = path_ip.points.at(stop_idx_ip).point.pose;
    *stop_line_idx = util::insertPoint(inserted_stop_point, path);
  }

  return true;
}


bool IntersectionModule::getObjectiveLanelets(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr, const int lane_id,
  std::vector<lanelet::ConstLanelet> * objective_lanelets)
{
  lanelet::ConstLanelet assigned_lanelet =
    lanelet_map_ptr->laneletLayer.get(lane_id);  // current assigned lanelets

  // get conflicting lanes on assigned lanelet
  const auto conflicting_lanelets =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);
  auto candidate_lanelets = conflicting_lanelets;

  // get previous lanelet of conflicting lanelets
  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    lanelet::ConstLanelets previous_lanelets = routing_graph_ptr->previous(conflicting_lanelet);
    for (const auto & previous_lanelet : previous_lanelets) {
      candidate_lanelets.push_back(previous_lanelet);
    }
  }

  // get lanelets that must be ignored
  std::vector<lanelet::ConstLanelet> exclude_lanelets;

  // for the behind ego-car lane.
  for (const auto & previous_lanelet : routing_graph_ptr->previous(assigned_lanelet)) {
    exclude_lanelets.push_back(previous_lanelet);
    for (const auto & following_lanelet : routing_graph_ptr->following(previous_lanelet)) {
      if (lanelet::utils::contains(exclude_lanelets, following_lanelet)) {
        continue;
      }
      exclude_lanelets.push_back(following_lanelet);
    }
  }

  // for non-priority roads.
  const auto right_of_ways = assigned_lanelet.regulatoryElementsAs<lanelet::RightOfWay>();
  for (const auto & right_of_way : right_of_ways) {
    for (const auto & yield_lanelets : right_of_way->yieldLanelets()) {
      exclude_lanelets.push_back(yield_lanelets);
      for (const auto & previous_lanelet : routing_graph_ptr->previous(yield_lanelets)) {
        exclude_lanelets.push_back(previous_lanelet);
      }
    }
  }

  // remove exclude_lanelets from candidates
  for (const auto & candidate_lanelet : candidate_lanelets) {
    if (lanelet::utils::contains(exclude_lanelets, candidate_lanelet)) {
      continue;
    }

    objective_lanelets->push_back(candidate_lanelet);
  }

  return true;
}

bool IntersectionModule::checkCollision(
  const autoware_planning_msgs::PathWithLaneId & path,
  const std::vector<lanelet::ConstLanelet> & objective_lanelets,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr, const double path_width)
{
  /* generates side edge line */
  autoware_planning_msgs::PathWithLaneId path_r;  // right side edge line
  autoware_planning_msgs::PathWithLaneId path_l;  // left side edge line
  generateEdgeLine(path, path_width, &path_r, &path_l);

  debug_data_.path_right_edge = path_r;
  debug_data_.path_left_edge = path_l;

  /* check collision for each objects and lanelets area */
  for (const auto & objective_lanelet : objective_lanelets) {
    for (const auto & object : objects_ptr->objects) {
      const auto object_pose = object.state.pose_covariance.pose;

      // TODO(Kenji Miyake): Map Matching of objects

      const bool is_in_objective_lanelet =
        bg::within(to_bg2d(object_pose.position), objective_lanelet.polygon2d());
      if (!is_in_objective_lanelet) {
        continue;
      }

      const bool is_in_path = bg::within(
        to_bg2d(object_pose.position),
        lines2polygon(to_bg2d(path_l.points), to_bg2d(path_r.points)));
      if (is_in_path) {
        // TODO(Kenji Miyake): check direction?
        continue;
      }

      const auto has_right_collision = checkPathCollision(path_r, object);
      const auto has_left_collision = checkPathCollision(path_l, object);
      if (!has_right_collision && !has_left_collision) {
        continue;
      }



      return true;
    }
  }

  return false;
}

bool IntersectionModule::checkPathCollision(
  const autoware_planning_msgs::PathWithLaneId & path,
  const autoware_perception_msgs::DynamicObject & object)
{
  for (const auto object_path : object.state.predicted_paths) {
    if (bg::intersects(to_bg2d(path.points), to_bg2d(object_path.path))) {
      return true;
    }
  }

  return false;
}

bool IntersectionModule::generateEdgeLine(
  const autoware_planning_msgs::PathWithLaneId & path, const double path_width,
  autoware_planning_msgs::PathWithLaneId * path_r, autoware_planning_msgs::PathWithLaneId * path_l)
{
  *path_r = path;
  *path_l = path;
  for (int i = 0; i < path.points.size(); ++i) {
    const double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    path_r->points.at(i).point.pose.position.x += path_width * std::sin(yaw);
    path_r->points.at(i).point.pose.position.y -= path_width * std::cos(yaw);
    path_l->points.at(i).point.pose.position.x -= path_width * std::sin(yaw);
    path_l->points.at(i).point.pose.position.y += path_width * std::cos(yaw);
  }
}

void IntersectionModule::StateMachine::setStateWithMarginTime(State state)
{
  /* same state request */
  if (state_ == state) {
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* GO -> STOP */
  if (state == State::STOP) {
    state_ = State::STOP;
    start_time_ = nullptr;  // reset timer
    return;
  }

  /* STOP -> GO */
  if (state == State::GO) {
    if (start_time_ == nullptr) {
      start_time_ = std::make_shared<ros::Time>(ros::Time::now());
      return;
    } else {
      const double duration = (ros::Time::now() - *start_time_).toSec();
      if (duration > margin_time_) {
        state_ = State::GO;
        start_time_ = nullptr;  // reset timer
      } else {
      }
      return;
    }
  }

  ROS_ERROR(
    "[StateMachine::setStateWithMarginTime()] : Unsuitable state. ignore "
    "request.");
  return;
}

void IntersectionModule::StateMachine::setState(State state) { state_ = state; }

void IntersectionModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

IntersectionModule::State IntersectionModule::StateMachine::getState() { return state_; }

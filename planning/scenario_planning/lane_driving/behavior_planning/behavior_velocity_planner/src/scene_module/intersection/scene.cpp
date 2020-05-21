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
#include <lanelet2_extension/regulatory_elements/road_marking.h>
#include <lanelet2_extension/utility/query.h>

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
  std::vector<lanelet::CompoundPolygon3d> objective_polygons;
  getObjectivePolygons(lanelet_map_ptr, routing_graph_ptr, lane_id_, &objective_polygons);
  DEBUG_INFO(
    "[IntersectionModuleManager::run] lane_id = %ld, objective_polygons.size = %lu", lane_id_,
    objective_polygons.size());
  if (objective_polygons.empty()) {
    DEBUG_INFO("[IntersectionModule::run]: no detection area. skip computation.");
    return true;
  }

ROS_WARN("before path.size = %lu", path->points.size());
  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int judge_line_idx = -1;
  if (!generateStopLine(closest, objective_polygons, path, &stop_line_idx, &judge_line_idx)) {
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
  if (checkCollision(*path, objective_polygons, objects_ptr, path_expand_width_)) {
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

int IntersectionModule::getFirstPointInsidePolygons(
  const autoware_planning_msgs::PathWithLaneId & path,
  const std::vector<lanelet::CompoundPolygon3d> & polygons) const
{
  int first_idx_inside_lanelet = -1;
  for (size_t i = 0; i < path.points.size(); ++i) {
    bool is_in_lanelet = false;
    auto p = path.points.at(i).point.pose.position;
    for (const auto & polygon : polygons) {
      const auto polygon_2d = lanelet::utils::to2D(polygon);
      is_in_lanelet = bg::within(to_bg2d(p), polygon_2d);
      if (is_in_lanelet) {
        first_idx_inside_lanelet = static_cast<int>(i);
        break;
      }
    }
    if (is_in_lanelet) break;
  }
  return first_idx_inside_lanelet;
}

bool IntersectionModule::generateStopLine(
  const int closest, const std::vector<lanelet::CompoundPolygon3d> objective_polygons,
  autoware_planning_msgs::PathWithLaneId * path, int * stop_line_idx, int * judge_line_idx) const
{
  /* set judge line dist */
  const double current_vel = planner_data_->current_velocity->twist.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold_;
  const double judge_line_dist = planning_utils::calcJudgeLineDist(current_vel, max_acc, 0.0);

  /* set parameters */
  constexpr double interval = 0.2;
  constexpr double mergin_dist_ = 1.0;
  const int mergin_idx_dist = std::ceil(mergin_dist_ / interval);
  const int base2front_idx_dist = std::ceil(planner_data_->base_link2front / interval);
  const int judge_idx_dist = std::ceil(judge_line_dist / interval);

  /* spline interpolation */
  autoware_planning_msgs::PathWithLaneId path_ip;
  if (!util::splineInterpolate(*path, interval, &path_ip)) return false;
  debug_data_.spline_path = path_ip;

  /* generate stop point */
  // If a stop_line is defined in lanelet_map, use it.
  // else, generates a local stop_line with considering the lane conflictions.
  int stop_idx_ip;  // stop point index for interpolated path.
  geometry_msgs::Point stop_point_from_map;
  if (getStopPoseFromMap(lane_id_, &stop_point_from_map)) {
    planning_utils::calcClosestIndex(path_ip, stop_point_from_map, stop_idx_ip, 10.0);
    stop_idx_ip = std::max(stop_idx_ip - base2front_idx_dist, 0);
  } else {
    int first_idx_inside_lane = getFirstPointInsidePolygons(path_ip, objective_polygons);
    if (first_idx_inside_lane == -1) {
      DEBUG_INFO("[generateStopLine] no stop line exist.");
      return false;
    }
    stop_idx_ip = std::max(first_idx_inside_lane - 1 - mergin_idx_dist - base2front_idx_dist, 0);
  }

  // insert stop_point
  const auto inserted_stop_point = path_ip.points.at(stop_idx_ip).point.pose;
  *stop_line_idx = util::insertPoint(inserted_stop_point, path);

  // if another stop point exist before intersection stop_line, disable judge_line.
  bool stop_point_exists_before_intersection_stop_line = false;
  for (int i = 0; i < *stop_line_idx; ++i) {
    if (std::fabs(path->points.at(i).point.twist.linear.x) < 0.1) {
      stop_point_exists_before_intersection_stop_line = true;
      break;
    }
  }

  /* insert judge point */
  const int judge_idx_ip = std::max(stop_idx_ip - judge_idx_dist, 0);
  ROS_INFO("[generateStopLine] stop_idx_ip = %d, judge_idx_ip = %d, mergin_dist = %f, mergin_idx_dist = %d",
      stop_idx_ip, judge_idx_ip, mergin_dist_, mergin_idx_dist);
  if (stop_point_exists_before_intersection_stop_line || stop_idx_ip == judge_idx_ip) {
    *judge_line_idx = *stop_line_idx;
    DEBUG_INFO("[generateStopLine] judge distance is zero.");
  } else {
    const auto inserted_judge_point = path_ip.points.at(judge_idx_ip).point.pose;
    *judge_line_idx = util::insertPoint(inserted_judge_point, path);
    ++(*stop_line_idx);  // stop index is incremented by judge line insertion
  }
  ROS_INFO("[generateStopLine] stop_line_idx = %d, judge_line_idx = %d, line = %d", *stop_line_idx, *judge_line_idx, __LINE__);

  return true;
}

bool IntersectionModule::getObjectivePolygons(
  lanelet::LaneletMapConstPtr lanelet_map_ptr,
  lanelet::routing::RoutingGraphPtr routing_graph_ptr, const int lane_id,
  std::vector<lanelet::CompoundPolygon3d> * polygons)
{
  const auto & assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);

  lanelet::ConstLanelets exclude_lanelets;

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

  // get conflicting lanes on assigned lanelet
  const auto & conflicting_lanelets =
    lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);

  lanelet::ConstLanelets objective_lanelets;  // final objective lanelets

  // remove exclude_lanelets from candidates
  for (const auto & conflicting_lanelet : conflicting_lanelets) {
    if (lanelet::utils::contains(exclude_lanelets, conflicting_lanelet)) {
      continue;
    }
    objective_lanelets.push_back(conflicting_lanelet);
  }

  // get possible lanelet path that reaches conflicting_lane longer than given length
  double length = 100;
  std::vector<lanelet::ConstLanelets> objective_lanelets_sequences;
  for (const auto & ll : objective_lanelets) {
    const auto & lanelet_sequences =
      lanelet::utils::query::getPreceedingLaneletSequences(routing_graph_ptr, ll, length);
    for (const auto & l : lanelet_sequences) {
      objective_lanelets_sequences.push_back(l);
    }
  }

  // get exact polygon of interest with exact length
  polygons->clear();
  for (const auto & ll : objective_lanelets_sequences) {
    const double path_length = lanelet::utils::getLaneletLength3d(ll);
    const auto polygon3d =
      lanelet::utils::getPolygonFromArcLength(ll, path_length - length, path_length);
    polygons->push_back(polygon3d);
  }

  debug_data_.detection_area = *polygons;

  return true;
}

bool IntersectionModule::checkCollision(
  const autoware_planning_msgs::PathWithLaneId & path,
  const std::vector<lanelet::CompoundPolygon3d> & objective_polygons,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr, const double path_width)
{
  /* generates side edge line */
  autoware_planning_msgs::PathWithLaneId path_r;  // right side edge line
  autoware_planning_msgs::PathWithLaneId path_l;  // left side edge line
  generateEdgeLine(path, path_width, &path_r, &path_l);

  debug_data_.path_right_edge = path_r;
  debug_data_.path_left_edge = path_l;

  /* check collision for each objects and lanelets area */
  for (const auto & objective_polygon : objective_polygons) {
    for (const auto & object : objects_ptr->objects) {
      const auto object_pose = object.state.pose_covariance.pose;

      // TODO(Kenji Miyake): Map Matching of objects

      const auto objective_polygon_2d = lanelet::utils::to2D(objective_polygon);
      const bool is_in_objective_lanelet =
        bg::within(to_bg2d(object_pose.position), objective_polygon_2d);
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

bool IntersectionModule::getStopPoseFromMap(
  const int lane_id, geometry_msgs::Point * stop_point) const
{
  lanelet::ConstLanelet lanelet = planner_data_->lanelet_map->laneletLayer.get(lane_id);
  const auto road_markings = lanelet.regulatoryElementsAs<lanelet::autoware::RoadMarking>();
  lanelet::ConstLineStrings3d stop_line;
  for (const auto & road_marking : road_markings) {
    const std::string type =
      road_marking->roadMarking().attributeOr(lanelet::AttributeName::Type, "none");
    if (type == lanelet::AttributeValueString::StopLine) {
      stop_line.push_back(road_marking->roadMarking());
      break;  // only one stop_line exists.
    }
  }
  if (stop_line.empty()) {
    ROS_INFO("map has no stop_line.");
    return false;
  }

  const auto p_start = stop_line.front().front();
  const auto p_end = stop_line.front().back();
  stop_point->x = 0.5 * (p_start.x() + p_end.x());
  stop_point->y = 0.5 * (p_start.y() + p_end.y());
  stop_point->z = 0.5 * (p_start.z() + p_end.z());

  return true;
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
    } else {
      const double duration = (ros::Time::now() - *start_time_).toSec();
      if (duration > margin_time_) {
        state_ = State::GO;
        start_time_ = nullptr;  // reset timer
      }
    }
    return;
  }

  ROS_ERROR("[StateMachine] : Unsuitable state. ignore request.");
  return;
}

void IntersectionModule::StateMachine::setState(State state) { state_ = state; }

void IntersectionModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

IntersectionModule::State IntersectionModule::StateMachine::getState() { return state_; }

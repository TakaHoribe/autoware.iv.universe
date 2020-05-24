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
#include <lanelet2_extension/regulatory_elements/road_marking.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>

#include "scene_module/intersection/util.h"
#include "utilization/boost_geometry_helper.h"
#include "utilization/interpolate.h"
#include "utilization/util.h"

// clang-format off
#define DEBUG_INFO(...) { ROS_INFO_COND(show_debug_info_, __VA_ARGS__); }

// clang-format on
namespace bg = boost::geometry;

IntersectionModule::IntersectionModule(
  const int64_t module_id, const int64_t lane_id, std::shared_ptr<const PlannerData> planner_data)
: SceneModuleInterface(module_id), lane_id_(lane_id)
{
  state_machine_.setMarginTime(2.0);  // [sec]
  show_debug_info_ = true;
  path_expand_width_ = 2.0;
  stop_line_margin_ = 1.0;
  decel_velocoity_ = 30.0 / 3.6;
  stuck_vehicle_detect_dist_ = 5.0;
  stuck_vehicle_vel_thr_ = 3.0 / 3.6;


  const auto & assigned_lanelet = planner_data->lanelet_map->laneletLayer.get(lane_id);
  turn_direction_ = assigned_lanelet.attributeOr("turn_direction", "else");
  has_traffic_light_ =
    !(assigned_lanelet.regulatoryElementsAs<const lanelet::TrafficLight>().empty());
}

bool IntersectionModule::modifyPathVelocity(autoware_planning_msgs::PathWithLaneId * path)
{
  debug_data_ = {};

  const auto input_path = *path;
  debug_data_.path_raw = input_path;

  State current_state = state_machine_.getState();
  DEBUG_INFO("[Intersection] lane_id = %ld, state = %d", lane_id_, static_cast<int>(current_state));

  /* get current pose */
  geometry_msgs::PoseStamped current_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* get detection area */
  std::vector<lanelet::CompoundPolygon3d> objective_polygons;
  getObjectivePolygons(lanelet_map_ptr, routing_graph_ptr, lane_id_, &objective_polygons);
  if (objective_polygons.empty()) {
    DEBUG_INFO("[Intersection] no detection area. skip computation.");
    return true;
  }

  /* set stop-line and stop-judgement-line for base_link */
  int stop_line_idx = -1;
  int judge_line_idx = -1;
  if (!generateStopLine(objective_polygons, path, &stop_line_idx, &judge_line_idx)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModule::run] setStopLineIdx fail");
    return false;
  }

  if (stop_line_idx <= 0 || judge_line_idx <= 0) {
    DEBUG_INFO("[Intersection] stop line or judge line is at path[0], ignore planning.");
    return true;
  }

  /* calc closest index */
  int closest = -1;
  if (!planning_utils::calcClosestIndex(input_path, current_pose.pose, closest)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[Intersection] calcClosestIndex fail");
    return false;
  }

  debug_data_.virtual_wall_pose =
    util::getAheadPose(stop_line_idx, planner_data_->base_link2front, *path);
  debug_data_.stop_point_pose = path->points.at(stop_line_idx).point.pose;
  debug_data_.judge_point_pose = path->points.at(judge_line_idx).point.pose;

  /* if current_state = GO, and current_pose is in front of stop_line, ignore planning. */
  bool is_over_judge_line = static_cast<bool>(closest > judge_line_idx);
  if (closest == judge_line_idx) {
    geometry_msgs::Pose judge_line = path->points.at(judge_line_idx).point.pose;
    is_over_judge_line = util::isAheadOf(current_pose.pose, judge_line);
  }
  if (current_state == State::GO && is_over_judge_line) {
    DEBUG_INFO("[Intersection] over the judge line. no plan needed.");
    return true;  // no plan needed.
  }

  /* get dynamic object */
  const auto objects_ptr = planner_data_->dynamic_objects;

  /* calculate dynamic collision around detection area */
  bool has_collision = checkCollision(*path, objective_polygons, objects_ptr, closest);
  bool is_stuck = checkStuckVehicleInIntersection(*path, closest, objects_ptr);
  bool is_entry_prohibited = (has_collision || is_stuck);
  state_machine_.setStateWithMarginTime(is_entry_prohibited ? State::STOP : State::GO);

  /* set stop speed : TODO behavior on straight lane should be improved*/
  if (state_machine_.getState() == State::STOP) {
    constexpr double stop_vel = 0.0;
    double v = (has_traffic_light_ && turn_direction_ == "straight") ? decel_velocoity_ : stop_vel;
    util::setVelocityFrom(stop_line_idx, v, path);
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
  const std::vector<lanelet::CompoundPolygon3d> objective_polygons,
  autoware_planning_msgs::PathWithLaneId * path, int * stop_line_idx, int * judge_line_idx) const
{
  /* set judge line dist */
  const double current_vel = planner_data_->current_velocity->twist.linear.x;
  const double max_acc = planner_data_->max_stop_acceleration_threshold_;
  const double judge_line_dist = planning_utils::calcJudgeLineDist(current_vel, max_acc, 0.0);

  /* set parameters */
  constexpr double interval = 0.2;
  const int margin_idx_dist = std::ceil(stop_line_margin_ / interval);
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
      DEBUG_INFO("[intersection] generate stopline, but no intersect line found.");
      return false;
    }
    stop_idx_ip = std::max(first_idx_inside_lane - 1 - margin_idx_dist - base2front_idx_dist, 0);
  }

  /* insert stop_point */
  const auto inserted_stop_point = path_ip.points.at(stop_idx_ip).point.pose;
  *stop_line_idx = util::insertPoint(inserted_stop_point, path);

  /* if another stop point exist before intersection stop_line, disable judge_line. */
  bool has_prior_stopline = false;
  for (int i = 0; i < *stop_line_idx; ++i) {
    if (std::fabs(path->points.at(i).point.twist.linear.x) < 0.1) {
      has_prior_stopline = true;
      break;
    }
  }

  /* insert judge point */
  const int judge_idx_ip = std::max(stop_idx_ip - judge_idx_dist, 0);
  if (has_prior_stopline || stop_idx_ip == judge_idx_ip) {
    *judge_line_idx = *stop_line_idx;
  } else {
    const auto inserted_judge_point = path_ip.points.at(judge_idx_ip).point.pose;
    *judge_line_idx = util::insertPoint(inserted_judge_point, path);
    ++(*stop_line_idx);  // stop index is incremented by judge line insertion
  }

  DEBUG_INFO(
    "[intersection] generateStopLine() : stop_idx = %d, judge_idx = %d, stop_idx_ip = %d, "
    "judge_idx_ip = %d, has_prior_stopline = %d",
    *stop_line_idx, *judge_line_idx, stop_idx_ip, judge_idx_ip, has_prior_stopline);

  return true;
}

bool IntersectionModule::getObjectivePolygons(
  lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphPtr routing_graph_ptr,
  const int lane_id, std::vector<lanelet::CompoundPolygon3d> * polygons)
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

void IntersectionModule::cutPredictPathWithDuration(
  autoware_perception_msgs::DynamicObjectArray * objects_ptr, const double time_thr) const
{
  const ros::Time current_time = ros::Time::now();
  for (auto & object : objects_ptr->objects) {                        // each objects
    for (auto & predicted_path : object.state.predicted_paths) {  // each predicted paths
      std::vector<geometry_msgs::PoseWithCovarianceStamped> vp;
      for (auto & predicted_pose : predicted_path.path) {  // each path points
        if ((predicted_pose.header.stamp - current_time).toSec() < time_thr) {
          vp.push_back(predicted_pose);
        }
      }
      predicted_path.path = vp;
    }
  }
}

bool IntersectionModule::checkCollision(
  const autoware_planning_msgs::PathWithLaneId & path,
  const std::vector<lanelet::CompoundPolygon3d> & objective_polygons,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr, const int closest)
{
  const Polygon2d ego_poly =
    generateEgoIntersectionLanePolygon(path, closest, 0.0);  // TODO use Lanelet
  debug_data_.ego_lane_polygon = toGeomMsg(ego_poly);

  /* extruct objects in detection area */
  autoware_perception_msgs::DynamicObjectArray objects_in_detection_area;
  for (const auto & object : objects_ptr->objects) {
    const auto object_pose = object.state.pose_covariance.pose;
    const bool is_in_ego_lane = bg::within(to_bg2d(object_pose.position), ego_poly);
    if (is_in_ego_lane) {
      ROS_INFO("[intersection] object is in ego_lane. ignored.");
      continue;  // TODO(Kenji Miyake): check direction?
    }

    for (const auto & objective_polygon : objective_polygons) {
      const bool is_in_objective_lanelet =
        bg::within(to_bg2d(object_pose.position), lanelet::utils::to2D(objective_polygon));
      if (is_in_objective_lanelet) {
        objects_in_detection_area.objects.push_back(object);
        break;
      }
    }
  }
  ROS_INFO(
    "[intersection] object_raw.size = %lu, object_in_DA.size = %lu.", objects_ptr->objects.size(),
    objects_in_detection_area.objects.size());

  /* remove the predicted position after passing_time */
  const double passing_time = calcIntersectionPassingTime(path, closest, lane_id_);
  cutPredictPathWithDuration(&objects_in_detection_area, passing_time);

  /* check collision between predicted_path and ego_area */
  int dc = 0;

  bool collision_detected = false;
  for (const auto & object : objects_in_detection_area.objects) {
    bool has_collision = false;
    for (const auto & predicted_path : object.state.predicted_paths) {
      has_collision = bg::intersects(ego_poly, to_bg2d(predicted_path.path));
      if (has_collision) {
        collision_detected = true;
        debug_data_.conflicting_targets.objects.push_back(object);
        break;
      }
    }
    if (has_collision) {
      ROS_INFO("[intersection] object no.%d has_collsion = YES!!!", dc++);
    } else {
      ROS_INFO("[intersection] object no.%d has_collsion = No", dc++);
    }
  }

  return collision_detected;
}

Polygon2d IntersectionModule::generateEgoIntersectionLanePolygon(
  const autoware_planning_msgs::PathWithLaneId & path, const int closest,
  const double extra_dist) const
{
  size_t assigned_lane_end_idx = 0;
  bool has_assigned_lane_id_prev = false;
  for (size_t i = 0; i < path.points.size(); ++i) {
    bool has_assigned_lane_id = util::hasLaneId(path.points.at(i), lane_id_);
    if (has_assigned_lane_id_prev && !has_assigned_lane_id) {
      assigned_lane_end_idx = i;
      break;
    }
    has_assigned_lane_id_prev = has_assigned_lane_id;
  }

  size_t ego_area_end_idx = assigned_lane_end_idx;
  double dist_sum = 0.0;
  for (size_t i = assigned_lane_end_idx + 1; i < path.points.size(); ++i) {
    dist_sum += planning_utils::calcDist2d(path.points.at(i), path.points.at(i - 1));
    if (dist_sum > extra_dist) break;
    ++ego_area_end_idx;
  }

  Polygon2d ego_area;  // open polygon
  for (int i = closest; i <= ego_area_end_idx; ++i) {
    double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    double x = path.points.at(i).point.pose.position.x + path_expand_width_ * std::sin(yaw);
    double y = path.points.at(i).point.pose.position.y - path_expand_width_ * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
  }
  for (int i = ego_area_end_idx; i >= closest; --i) {
    double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    double x = path.points.at(i).point.pose.position.x - path_expand_width_ * std::sin(yaw);
    double y = path.points.at(i).point.pose.position.y + path_expand_width_ * std::cos(yaw);
    ego_area.outer().push_back(Point2d(x, y));
  }

  return ego_area;
}

double IntersectionModule::calcIntersectionPassingTime(
  const autoware_planning_msgs::PathWithLaneId & path, const int closest,
  const int objective_lane_id) const
{
  double dist_sum = 0.0;
  int assigned_lane_found = false;

  for (int i = closest + 1; i < path.points.size(); ++i) {
    dist_sum += planning_utils::calcDist2d(path.points.at(i - 1), path.points.at(i));
    bool has_objective_lane_id = util::hasLaneId(path.points.at(i), objective_lane_id);

    if (assigned_lane_found && !has_objective_lane_id) {
      break;
    }
    assigned_lane_found = has_objective_lane_id;
  }
  if (!assigned_lane_found) return 0.0;  // has already passed the intersection.

  constexpr double intersection_velocity = 3.0;  // TODO set to be reasonable
  const double passing_time = dist_sum / intersection_velocity;

  ROS_INFO("[intersection] dist = %f, passing_time = %f", dist_sum, passing_time);

  return passing_time;
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

bool IntersectionModule::checkStuckVehicleInIntersection(
  const autoware_planning_msgs::PathWithLaneId & path, const int closest,
  const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr) const
{
  const Polygon2d stuck_vehicle_detect_area =
    generateEgoIntersectionLanePolygon(path, closest, stuck_vehicle_detect_dist_);
  debug_data_.stuck_vehicle_detect_area = toGeomMsg(stuck_vehicle_detect_area);

  for (const auto & object : objects_ptr->objects) {
    if (!isTargetVehicleType(object)) {
      continue;  // not target vehicle type
    }
    if (std::fabs(object.state.twist_covariance.twist.linear.x) > stuck_vehicle_vel_thr_) {
      continue;  // not stop vehicle
    }
    const auto object_pos = object.state.pose_covariance.pose.position;
    if (bg::within(to_bg2d(object_pos), stuck_vehicle_detect_area)) {
      DEBUG_INFO("[intersection] stuck vehicle found.");
      debug_data_.stuck_targets.objects.push_back(object);
      return true;
    }
  }
  return false;
}

bool IntersectionModule::isTargetVehicleType(
  const autoware_perception_msgs::DynamicObject & object) const
{
  if (
    object.semantic.type == autoware_perception_msgs::Semantic::CAR ||
    object.semantic.type == autoware_perception_msgs::Semantic::BUS ||
    object.semantic.type == autoware_perception_msgs::Semantic::TRUCK) {
    return true;
  }
  return false;
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

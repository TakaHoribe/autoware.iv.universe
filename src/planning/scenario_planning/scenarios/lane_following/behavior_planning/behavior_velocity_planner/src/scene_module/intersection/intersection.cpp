#include <behavior_velocity_planner/api.hpp>
#include <scene_module/intersection/intersection.hpp>

#include "utilization/boost_geometry_helper.h"
#include "utilization/marker_helper.h"
#include "utilization/util.h"

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_extension/utility/utilities.h>

namespace behavior_planning {

namespace bg = boost::geometry;

IntersectionModule::IntersectionModule(const int lane_id, IntersectionModuleManager* intersection_module_manager)
    : assigned_lane_id_(lane_id), intersection_module_manager_(intersection_module_manager) {
  judge_line_dist_ = 1.5;                        // [m]
  approaching_speed_to_stopline_ = 100.0 / 3.6;  // 100[km/h]
  state_machine_.setMarginTime(2.0);             // [sec]
  path_expand_width_ = 2.0;
  show_debug_info_ = false;
}

bool IntersectionModule::run(const autoware_planning_msgs::PathWithLaneId& input,
                             autoware_planning_msgs::PathWithLaneId& output) {
  output = input;
  intersection_module_manager_->debugger_.publishPath(output, "path_raw", 0.0, 1.0, 1.0);

  State current_state = state_machine_.getState();
  ROS_DEBUG_COND(show_debug_info_, "[IntersectionModule]: run: state_machine_.getState() = %d", (int)current_state);

  /* get current pose */
  geometry_msgs::PoseStamped current_pose;
  if (!getCurrentSelfPose(current_pose)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModule::run] getCurrentSelfPose fail");
    return false;
  }

  /* check if the current_pose is ahead from judgement line */
  int closest = -1;
  if (!planning_utils::calcClosestIndex(output, current_pose.pose, closest)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModule::run] calcClosestIndex fail");
    return false;
  }

  /* set stop-line and stop-judgement-line */
  if (!setStopLineIdx(closest, judge_line_dist_, &output, &stop_line_idx_, &judge_line_idx_)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModule::run] setStopLineIdx fail");
    return false;
  }

  if (stop_line_idx_ <= 0 || judge_line_idx_ <= 0) {
    ROS_INFO_COND(show_debug_info_,
                  "[IntersectionModule::run] the stop line or judge line is at path[0], ignore "
                  "planning. Maybe it is far behind the current position.");
    return true;
  }
  intersection_module_manager_->debugger_.publishPose(output.points.at(stop_line_idx_).point.pose, "stop_point_pose",
                                                      1.0, 0.0, 0.0, static_cast<int>(current_state));
  intersection_module_manager_->debugger_.publishPose(output.points.at(judge_line_idx_).point.pose, "judge_point_pose",
                                                      1.0, 1.0, 0.5, static_cast<int>(current_state));
  intersection_module_manager_->debugger_.publishPath(output, "path_with_judgeline", 0.0, 0.5, 1.0);

  /* set approaching speed to stop-line */
  setVelocityFrom(judge_line_idx_, approaching_speed_to_stopline_, &output);

  if (current_state == State::GO) {
    geometry_msgs::Pose p =
        planning_utils::transformRelCoordinate2D(current_pose.pose, output.points.at(judge_line_idx_).point.pose);
    // current_pose is ahead of judge_line
    if (p.position.x > 0.0) {
      ROS_INFO_COND(show_debug_info_, "[IntersectionModule::run] no plan needed. skip collision check.");
      return true;  // no plan needed.
    }
  }

  /* get lanelet map */
  lanelet::LaneletMapConstPtr lanelet_map_ptr;               // objects info
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;  // route info
  if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModuleManager::run()] cannot get lanelet map");
    return false;
  }

  /* get detection area */
  std::vector<lanelet::ConstLanelet> objective_lanelets;
  getObjectiveLanelets(lanelet_map_ptr, routing_graph_ptr, assigned_lane_id_, &objective_lanelets);
  intersection_module_manager_->debugger_.publishLaneletsArea(objective_lanelets, "intersection_detection_lanelets");
  ROS_DEBUG_COND(show_debug_info_,
                 "[IntersectionModuleManager::run()] assigned_lane_id_ = %d, objective_lanelets.size() = %lu",
                 assigned_lane_id_, objective_lanelets.size());
  if (objective_lanelets.empty()) {
    ROS_DEBUG_COND(show_debug_info_, "[IntersectionModule::run]: detection area number is zero. skip computation.");
    return true;
  }

  /* get dynamic object */
  auto objects_ptr = std::make_shared<const autoware_perception_msgs::DynamicObjectArray>();
  if (!getDynemicObjects(objects_ptr)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModuleManager::run()] cannot get dynamic object");
    return false;
  }

  /* calculate dynamic collision around detection area */
  if (checkCollision(output, objective_lanelets, objects_ptr, path_expand_width_)) {
    state_machine_.setStateWithMarginTime(State::STOP);
  } else {
    state_machine_.setStateWithMarginTime(State::GO);
  }

  /* set stop speed */
  if (state_machine_.getState() == State::STOP) {
    const double stop_vel = 0.0;
    setVelocityFrom(stop_line_idx_, stop_vel, &output);
  }

  return true;
}

bool IntersectionModule::endOfLife(const autoware_planning_msgs::PathWithLaneId& input) {
  /* search if the assigned lane_id is still exists */
  bool is_assigned_lane_id_found = false;
  for (const auto& point : input.points) {
    for (const auto& id : point.lane_ids) {
      if (assigned_lane_id_ == id) {
        is_assigned_lane_id_found = true;
        break;
      }
    }
    if (is_assigned_lane_id_found == true) {
      break;
    }
  }

  bool is_end_of_life = !is_assigned_lane_id_found;

  if (is_end_of_life) {
    intersection_module_manager_->unregisterTask(assigned_lane_id_);
  }

  return is_end_of_life;
}

bool IntersectionModule::setStopLineIdx(const int current_pose_closest, const double judge_line_dist,
                                        autoware_planning_msgs::PathWithLaneId* path, int* stop_line_idx,
                                        int* judge_line_idx) {
  // TEMP: return first assigned_lane_id point's index
  *stop_line_idx = -1;
  for (size_t i = 0; i < path->points.size(); ++i) {
    for (const auto& id : path->points.at(i).lane_ids) {
      if (id == assigned_lane_id_) {
        *stop_line_idx = i;
      }
      if (*stop_line_idx != -1) break;
    }
    if (*stop_line_idx != -1) break;
  }

  if (*stop_line_idx == -1) {
    ROS_ERROR(
        "[IntersectionModule::setStopLineIdx]: cannot set the stop line. something wrong. please "
        "check code. ");
    return false;  // cannot find stop line.
  }

  // TEMP: should use interpolation (points distance may be very long)
  double curr_dist = 0.0;
  double prev_dist = curr_dist;
  *judge_line_idx = -1;
  for (size_t i = *stop_line_idx; i > 0; --i) {
    const geometry_msgs::Point p0 = path->points.at(i).point.pose.position;
    const geometry_msgs::Point p1 = path->points.at(i - 1).point.pose.position;
    curr_dist += planning_utils::calcDist2d(p0, p1);
    if (curr_dist > judge_line_dist) {
      const double dl = std::max(curr_dist - prev_dist, 0.0001 /* avoid 0 divide */);
      const double w_p0 = (curr_dist - judge_line_dist) / dl;
      const double w_p1 = (judge_line_dist - prev_dist) / dl;
      autoware_planning_msgs::PathPointWithLaneId p = path->points.at(i);
      p.point.pose.position.x = w_p0 * p0.x + w_p1 * p1.x;
      p.point.pose.position.y = w_p0 * p0.y + w_p1 * p1.y;
      p.point.pose.position.z = w_p0 * p0.z + w_p1 * p1.z;
      auto itr = path->points.begin();
      itr += i;
      path->points.insert(itr, p);
      *judge_line_idx = i;
      ++stop_line_idx;
      break;
    }
    prev_dist = curr_dist;
  }
  if (*judge_line_idx == -1) {
    ROS_DEBUG(
        "[IntersectionModule::setStopLineIdx]: cannot set the stop judgement line. path is too "
        "short, or "
        "the vehicle is already ahead of the stop line. stop_line_id = %d",
        *stop_line_idx);
  }
  return true;
}

bool IntersectionModule::setVelocityFrom(const size_t idx, const double vel,
                                         autoware_planning_msgs::PathWithLaneId* input) {
  for (size_t i = idx; i < input->points.size(); ++i) {
    input->points.at(i).point.twist.linear.x = std::min(vel, input->points.at(i).point.twist.linear.x);
  }
}

bool IntersectionModule::getObjectiveLanelets(lanelet::LaneletMapConstPtr lanelet_map_ptr,
                                              lanelet::routing::RoutingGraphConstPtr routing_graph_ptr,
                                              const int lane_id,
                                              std::vector<lanelet::ConstLanelet>* objective_lanelets) {
  lanelet::ConstLanelet assigned_lanelet = lanelet_map_ptr->laneletLayer.get(lane_id);  // current assigned lanelets

  // get conflicting lanes on assigned lanelet
  *objective_lanelets = lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);

  // get previous lanelet of conflicting lanelets
  const size_t conflicting_lanelets_num = objective_lanelets->size();
  for (size_t i = 0; i < conflicting_lanelets_num; ++i) {
    lanelet::ConstLanelets previous_lanelets = routing_graph_ptr->previous(objective_lanelets->at(i));
    for (size_t j = 0; j < previous_lanelets.size(); ++j) {
      objective_lanelets->push_back(previous_lanelets.at(j));
    }
  }
  return true;
}

bool IntersectionModule::checkCollision(
    const autoware_planning_msgs::PathWithLaneId& path, const std::vector<lanelet::ConstLanelet>& objective_lanelets,
    const std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects_ptr, const double path_width) {
  /* generates side edge line */
  autoware_planning_msgs::PathWithLaneId path_r;  // right side edge line
  autoware_planning_msgs::PathWithLaneId path_l;  // left side edge line
  generateEdgeLine(path, path_width, &path_r, &path_l);

  /* check collision for each objects and lanelets area */
  bool is_collision_detected = false;
  for (size_t i = 0; i < objective_lanelets.size(); ++i) {
    const auto objective_lanelet = objective_lanelets.at(i);

    for (size_t j = 0; j < objects_ptr->objects.size(); ++j) {
      const auto position = objects_ptr->objects.at(j).state.pose_covariance.pose.position;
      if (bg::within(to_bg2d(position), objective_lanelet.polygon2d())) {
        if (checkPathCollision(path_r, objects_ptr->objects.at(j)) ||
            checkPathCollision(path_l, objects_ptr->objects.at(j))) {
          is_collision_detected = true;
        }
      }

      if (is_collision_detected) break;
    }
    if (is_collision_detected) break;
  }

  /* for debug */
  intersection_module_manager_->debugger_.publishPath(path_r, "path_right_edge", 0.5, 0.0, 0.5);
  intersection_module_manager_->debugger_.publishPath(path_l, "path_left_edge", 0.0, 0.5, 0.5);

  return is_collision_detected;
}

bool IntersectionModule::checkPathCollision(const autoware_planning_msgs::PathWithLaneId& path,
                                            const autoware_perception_msgs::DynamicObject& object) {
  for (const auto object_path : object.state.predicted_paths) {
    if (bg::intersects(to_bg2d(path.points), to_bg2d(object_path.path))) {
      return true;
    }
  }

  return false;
}

bool IntersectionModule::generateEdgeLine(const autoware_planning_msgs::PathWithLaneId& path, const double path_width,
                                          autoware_planning_msgs::PathWithLaneId* path_r,
                                          autoware_planning_msgs::PathWithLaneId* path_l) {
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

void IntersectionModule::StateMachine::setStateWithMarginTime(IntersectionModule::State state) {
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
      "[IntersectionModule::StateMachine::setStateWithMarginTime()] : Unsuitable state. ignore "
      "request.");
  return;
}

void IntersectionModule::StateMachine::setState(IntersectionModule::State state) { state_ = state; }

void IntersectionModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

IntersectionModule::State IntersectionModule::StateMachine::getState() { return state_; }

bool IntersectionModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                               std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  /* get self pose */
  geometry_msgs::PoseStamped self_pose;
  if (!getCurrentSelfPose(self_pose)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModuleManager::startCondition()] cannot get current self pose");
    return false;
  }

  /* get lanelet map */
  lanelet::LaneletMapConstPtr lanelet_map_ptr;               // objects info
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;  // route info
  if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModuleManager::startCondition()] cannot get lanelet map");
    return false;
  }

  /* search intersection tag */
  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      const int lane_id = input.points.at(i).lane_ids.at(j);
      lanelet::ConstLanelet lanelet_ij = lanelet_map_ptr->laneletLayer.get(lane_id);  // get lanelet layer
      std::string turn_direction = lanelet_ij.attributeOr("turn_direction", "else");  // get turn_direction

      if (!isRunning(lane_id)) {
        // check intersection tag
        if (turn_direction.compare("right") == 0 || turn_direction.compare("left") == 0 ||
            turn_direction.compare("straight") == 0) {
          // intersection tag is found. set module.
          v_module_ptr.push_back(std::make_shared<IntersectionModule>(lane_id, this));
          registerTask(lane_id);
        }
      }
    }
  }

  return true;
}

bool IntersectionModuleManager::isRunning(const int lane_id) {
  for (const auto& id : registered_lane_ids_) {
    if (id == lane_id) return true;
  }
  return false;
}

void IntersectionModuleManager::registerTask(const int lane_id) { registered_lane_ids_.push_back(lane_id); }

void IntersectionModuleManager::unregisterTask(const int lane_id) {
  const auto itr = std::find(registered_lane_ids_.begin(), registered_lane_ids_.end(), lane_id);
  if (itr == registered_lane_ids_.end())
    ROS_ERROR(
        "[IntersectionModuleManager::unregisterTask()] : cannot remove task (lane_id = %d,"
        " registered_lane_ids_.size() = %lu)",
        lane_id, registered_lane_ids_.size());
  registered_lane_ids_.erase(itr);
}

IntersectionModuleDebugger::IntersectionModuleDebugger() : nh_(""), pnh_("~") {
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/intersection", 20);
}

void IntersectionModuleDebugger::publishLaneletsArea(const std::vector<lanelet::ConstLanelet>& lanelets,
                                                     const std::string& ns) {
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  for (size_t i = 0; i < lanelets.size(); ++i) {
    lanelet::CompoundPolygon3d lanelet_i_polygon = lanelets.at(i).polygon3d();

    visualization_msgs::Marker marker{};
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = ns + "_" + std::to_string(i);
    marker.id = i;
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
    marker.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker.color = createMarkerColor(0.0, 1.0, 0.0, 0.999);
    for (size_t j = 0; j < lanelet_i_polygon.size(); ++j) {
      geometry_msgs::Point point;
      point.x = lanelet_i_polygon[j].x();
      point.y = lanelet_i_polygon[j].y();
      point.z = lanelet_i_polygon[j].z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }
  debug_viz_pub_.publish(msg);
}

void IntersectionModuleDebugger::publishPath(const autoware_planning_msgs::PathWithLaneId& path, const std::string& ns,
                                             const double r, const double g, const double b) {
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  for (int i = 0; i < path.points.size(); ++i) {
    marker.id = i;
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = path.points.at(i).point.pose;
    marker.scale = createMarkerScale(0.5, 0.3, 0.3);
    marker.color = createMarkerColor(r, g, b, 0.999);
    msg.markers.push_back(marker);
  }

  debug_viz_pub_.publish(msg);
}

void IntersectionModuleDebugger::publishPose(const geometry_msgs::Pose& pose, const std::string& ns, const double r,
                                             const double g, const double b, const int mode) {
  const auto current_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker{};
  marker.header.frame_id = "map";
  marker.header.stamp = current_time;
  marker.ns = ns;

  marker.id = 0;
  marker.lifetime = ros::Duration(0.3);
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale = createMarkerScale(0.5, 0.3, 0.3);
  marker.color = createMarkerColor(r, g, b, 0.999);
  msg.markers.push_back(marker);

  // STOP
  if (mode == 0) {
    visualization_msgs::Marker marker_line{};
    marker_line.header.frame_id = "map";
    marker_line.header.stamp = current_time;
    marker_line.ns = ns + "_line";
    marker_line.id = 1;
    marker_line.lifetime = ros::Duration(0.3);
    marker_line.type = visualization_msgs::Marker::LINE_STRIP;
    marker_line.action = visualization_msgs::Marker::ADD;
    marker_line.pose.orientation = createMarkerOrientation(0, 0, 0, 1.0);
    marker_line.scale = createMarkerScale(0.1, 0.0, 0.0);
    marker_line.color = createMarkerColor(r, g, b, 0.999);

    const double yaw = tf2::getYaw(pose.orientation);

    const double a = 3.0;
    geometry_msgs::Point p0;
    p0.x = pose.position.x - a * std::sin(yaw);
    p0.y = pose.position.y + a * std::cos(yaw);
    p0.z = pose.position.z;
    marker_line.points.push_back(p0);

    geometry_msgs::Point p1;
    p1.x = pose.position.x + a * std::sin(yaw);
    p1.y = pose.position.y - a * std::cos(yaw);
    p1.z = pose.position.z;
    marker_line.points.push_back(p1);

    msg.markers.push_back(marker_line);
  }

  debug_viz_pub_.publish(msg);
}

}  // namespace behavior_planning
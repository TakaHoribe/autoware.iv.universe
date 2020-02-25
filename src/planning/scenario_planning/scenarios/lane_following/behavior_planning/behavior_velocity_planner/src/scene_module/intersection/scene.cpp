#include <scene_module/intersection/scene.hpp>

#include <lanelet2_core/geometry/Polygon.h>
#include <lanelet2_extension/utility/utilities.h>

#include <behavior_velocity_planner/api.hpp>  // To be refactored
#include <scene_module/intersection/manager.hpp>

#include "utilization/boost_geometry_helper.h"
#include "utilization/util.h"

namespace behavior_planning {

namespace bg = boost::geometry;

IntersectionModule::IntersectionModule(const int lane_id, IntersectionModuleManager* intersection_module_manager)
    : assigned_lane_id_(lane_id), intersection_module_manager_(intersection_module_manager) {
  judge_line_dist_ = 1.5;                        // [m]
  approaching_speed_to_stopline_ = 100.0 / 3.6;  // 100[km/h]
  state_machine_.setMarginTime(2.0);             // [sec]
  path_expand_width_ = 2.0;
  show_debug_info_ = false;

  if (!getBaselink2FrontLength(baselink_to_front_length_)) {
    ROS_WARN("[IntersectionModule] : cannot get vehicle front to base_link. set default.");
    baselink_to_front_length_ = 5.0;
  }
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

  /* set stop-line and stop-judgement-line for base_link */
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

  if (current_state == State::STOP) {
    // visualize geofence at vehicle front position
    intersection_module_manager_->debugger_.publishGeofence(
        getAheadPose(stop_line_idx_, baselink_to_front_length_, output), assigned_lane_id_);
  }
  intersection_module_manager_->debugger_.publishPose(output.points.at(stop_line_idx_).point.pose, "stop_point_pose",
                                                      1.0, 0.0, 0.0, static_cast<int>(current_state));
  intersection_module_manager_->debugger_.publishPose(output.points.at(judge_line_idx_).point.pose, "judge_point_pose",
                                                      1.0, 1.0, 0.5, static_cast<int>(current_state));
  intersection_module_manager_->debugger_.publishPath(output, "output_path", 0.0, 0.5, 1.0);

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

  // insert judge line with interpolation
  double curr_dist = 0.0;
  double prev_dist = curr_dist;
  *judge_line_idx = -1;
  for (size_t i = *stop_line_idx; i > 0; --i) {
    const geometry_msgs::Pose p0 = path->points.at(i).point.pose;
    const geometry_msgs::Pose p1 = path->points.at(i - 1).point.pose;
    curr_dist += planning_utils::calcDist2d(p0, p1);
    if (curr_dist > judge_line_dist) {
      const double dl = std::max(curr_dist - prev_dist, 0.0001 /* avoid 0 divide */);
      const double w_p0 = (curr_dist - judge_line_dist) / dl;
      const double w_p1 = (judge_line_dist - prev_dist) / dl;
      autoware_planning_msgs::PathPointWithLaneId p = path->points.at(i);
      p.point.pose.position.x = w_p0 * p0.position.x + w_p1 * p1.position.x;
      p.point.pose.position.y = w_p0 * p0.position.y + w_p1 * p1.position.y;
      p.point.pose.position.z = w_p0 * p0.position.z + w_p1 * p1.position.z;
      tf2::Quaternion q0_tf, q1_tf;
      tf2::fromMsg(p0.orientation, q0_tf);
      tf2::fromMsg(p1.orientation, q1_tf);
      p.point.pose.orientation = tf2::toMsg(q0_tf.slerp(q1_tf, w_p1));
      auto itr = path->points.begin();
      itr += i;
      path->points.insert(itr, p);
      *judge_line_idx = i;
      ++(*stop_line_idx);
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

geometry_msgs::Pose IntersectionModule::getAheadPose(const size_t start_idx, const double ahead_dist,
                                           const autoware_planning_msgs::PathWithLaneId& path) const {
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
  const auto conflicting_lanelets = lanelet::utils::getConflictingLanelets(routing_graph_ptr, assigned_lanelet);
  auto candidate_lanelets = conflicting_lanelets;

  // get previous lanelet of conflicting lanelets
  for (const auto& conflicting_lanelet : conflicting_lanelets) {
    lanelet::ConstLanelets previous_lanelets = routing_graph_ptr->previous(conflicting_lanelet);
    for (const auto& previous_lanelet : previous_lanelets) {
      candidate_lanelets.push_back(previous_lanelet);
    }
  }

  const auto isConnectedTo = [&routing_graph_ptr](const auto& from_lanelet, const auto& to_lanelet) {
    const auto following_lanelets = routing_graph_ptr->following(from_lanelet);
    return lanelet::utils::contains(following_lanelets, to_lanelet);
  };

  // get same-group lanelets of assigned lanelet
  std::vector<lanelet::ConstLanelet> same_group_lanelets;
  for (const auto& previous_lanelet : routing_graph_ptr->previous(assigned_lanelet)) {
    if (!isConnectedTo(previous_lanelet, assigned_lanelet)) {
      continue;
    }

    same_group_lanelets = routing_graph_ptr->following(previous_lanelet);
    break;
  }

  // Filter candidates
  for (const auto& candidate_lanelet : candidate_lanelets) {
    if (lanelet::utils::contains(same_group_lanelets, candidate_lanelet)) {
      continue;
    }

    objective_lanelets->push_back(candidate_lanelet);
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

  /* for debug */
  intersection_module_manager_->debugger_.publishPath(path_r, "path_right_edge", 0.5, 0.0, 0.5);
  intersection_module_manager_->debugger_.publishPath(path_l, "path_left_edge", 0.0, 0.5, 0.5);

  /* check collision for each objects and lanelets area */
  for (const auto& objective_lanelet : objective_lanelets) {
    for (const auto& object : objects_ptr->objects) {
      const auto object_pose = object.state.pose_covariance.pose;

      // TODO(Kenji Miyake): Map Matching of objects

      const auto is_in_objective_lanelet = bg::within(to_bg2d(object_pose.position), objective_lanelet.polygon2d());
      if (!is_in_objective_lanelet) {
        continue;
      }

      const auto has_right_collision = checkPathCollision(path_r, object);
      const auto has_left_collision = checkPathCollision(path_l, object);
      if (!has_right_collision && !has_left_collision) {
        continue;
      }

      const auto is_in_path =
          bg::within(to_bg2d(object_pose.position), lines2polygon(to_bg2d(path_l.points), to_bg2d(path_r.points)));
      if (is_in_path) {
        // TODO(Kenji Miyake): check direction?
        continue;
      }

      return true;
    }
  }

  return false;
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

}  // namespace behavior_planning

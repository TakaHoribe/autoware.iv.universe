#include <behavior_velocity_planner/api.hpp>
#include <scene_module/blind_spot/blind_spot.hpp>

#include "utilization/util.h"

// clang-format on
namespace behavior_planning {

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;

/*
 * ========================= BlindSpot Module =========================
 */
BlindSpotModule::BlindSpotModule(const int lane_id, const std::string& turn_direction,
                                 BlindSpotModuleManager* blind_spot_module_manager)
    : assigned_lane_id_(lane_id),
      turn_direction_(turn_direction),
      blind_spot_module_manager_(blind_spot_module_manager) {
  judge_line_dist_ = 1.5;             // [m]
  state_machine_.setMarginTime(2.0);  // [sec]
  path_expand_width_ = 2.0;
  show_debug_info_ = false;
  if (!getBaselink2FrontLength(baselink_to_front_length_)) {
    ROS_WARN("[IntersectionModule] : cannot get vehicle front to base_link. set default.");
    baselink_to_front_length_ = 5.0;
  }
}

bool BlindSpotModule::run(const autoware_planning_msgs::PathWithLaneId& input,
                          autoware_planning_msgs::PathWithLaneId& output) {
  output = input;
  blind_spot_module_manager_->debugger_.publishPath(output, "path_raw", 0.0, 1.0, 1.0);

  State current_state = state_machine_.getState();
  ROS_DEBUG_COND(show_debug_info_, "[BlindSpotModule]: run: state_machine_.getState() = %d", (int)current_state);

  /* get current pose */
  geometry_msgs::PoseStamped current_pose;
  if (!getCurrentSelfPose(current_pose)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[BlindSpotModule::run] getCurrentSelfPose fail");
    return false;
  }

  /* check if the current_pose is ahead from judgement line */
  int closest = -1;
  if (!planning_utils::calcClosestIndex(output, current_pose.pose, closest)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[BlindSpotModule::run] calcClosestIndex fail");
    return false;
  }

  /* set stop-line and stop-judgement-line */
  if (!setStopLineIdx(closest, judge_line_dist_, output, stop_line_idx_, judge_line_idx_)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[BlindSpotModule::run] setStopLineIdx fail");
    return false;
  }

  if (stop_line_idx_ <= 0 || judge_line_idx_ <= 0) {
    ROS_INFO_COND(show_debug_info_,
                  "[BlindSpotModule::run] the stop line or judge line is at path[0], ignore "
                  "planning. Maybe it is far behind the current position.");
    return true;
  }

  if (current_state == State::STOP) {
    // visualize geofence at vehicle front position
    blind_spot_module_manager_->debugger_.publishGeofence(
        getAheadPose(stop_line_idx_, baselink_to_front_length_, output), assigned_lane_id_);
  }
  blind_spot_module_manager_->debugger_.publishPose(output.points.at(stop_line_idx_).point.pose, "stop_point_pose", 1.0,
                                                    0.0, 0.0, (int)current_state);
  blind_spot_module_manager_->debugger_.publishPose(output.points.at(judge_line_idx_).point.pose, "judge_point_pose",
                                                    1.0, 1.0, 0.5, (int)current_state);
  blind_spot_module_manager_->debugger_.publishPath(output, "path_with_judgeline", 0.0, 0.5, 1.0);

  if (current_state == State::GO) {
    geometry_msgs::Pose p =
        planning_utils::transformRelCoordinate2D(current_pose.pose, output.points.at(judge_line_idx_).point.pose);
    if (p.position.x > 0.0)  // current_pose is ahead of judge_line
    {
      ROS_INFO_COND(show_debug_info_, "[BlindSpotModule::run] no plan needed. skip collision check.");
      return true;  // no plan needed.
    }
  }

  /* get detection area */
  std::vector<std::vector<geometry_msgs::Point>> detection_areas;
  generateDetectionArea(current_pose.pose, detection_areas);
  blind_spot_module_manager_->debugger_.publishDetectionArea(detection_areas, (int)current_state,
                                                             "blind_spot_detection_area");

  /* get dynamic object */
  std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects_ptr =
      std::make_shared<autoware_perception_msgs::DynamicObjectArray>();
  if (!getDynemicObjects(objects_ptr)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[BlindSpotModuleManager::run()] cannot get dynamic object");
    return false;
  }

  /* calculate dynamic collision around detection area */
  bool is_collision = false;
  if (!checkCollision(output, detection_areas, objects_ptr, path_expand_width_, is_collision)) {
    return false;
  }

  if (is_collision) {
    state_machine_.setStateWithMarginTime(State::STOP);
  } else {
    state_machine_.setStateWithMarginTime(State::GO);
  }

  /* set stop speed */
  if (state_machine_.getState() == State::STOP) {
    const double stop_vel = 0.0;
    setVelocityFrom(stop_line_idx_, stop_vel, output);
  }

  return true;
}

bool BlindSpotModule::generateDetectionArea(const geometry_msgs::Pose& current_pose,
                                            std::vector<std::vector<geometry_msgs::Point>>& detection_areas) {
  detection_areas.clear();
  std::vector<geometry_msgs::Point> blind_spot;

  double detection_width = 5.0;
  double detection_length_forward = 5.0;
  double detection_length_backward = 15.0;
  double vehicle_width = 3.0;
  double vw = vehicle_width * 0.5;

  if (turn_direction_.compare("right") == 0) {
    vw *= -1.0;
    detection_width *= -1.0;
  } else if (turn_direction_.compare("left") == 0) {
    // nothing to do.
  } else {
    ROS_WARN("blind spot detector is running, turn_direction_ = not right or left. (%s)", turn_direction_.c_str());
    return false;
  }
  geometry_msgs::Pose fl;
  fl.position.x = detection_length_forward;
  fl.position.y = vw;
  blind_spot.push_back(planning_utils::transformAbsCoordinate2D(fl, current_pose).position);
  geometry_msgs::Pose fr;
  fr.position.x = detection_length_forward;
  fr.position.y = vw + detection_width;
  blind_spot.push_back(planning_utils::transformAbsCoordinate2D(fr, current_pose).position);
  geometry_msgs::Pose rr;
  rr.position.x = -detection_length_backward;
  rr.position.y = vw + detection_width;
  blind_spot.push_back(planning_utils::transformAbsCoordinate2D(rr, current_pose).position);
  geometry_msgs::Pose rl;
  rl.position.x = -detection_length_backward;
  rl.position.y = vw;
  blind_spot.push_back(planning_utils::transformAbsCoordinate2D(rl, current_pose).position);
  detection_areas.push_back(blind_spot);
  return true;
}

bool BlindSpotModule::endOfLife(const autoware_planning_msgs::PathWithLaneId& input) {
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
    blind_spot_module_manager_->unregisterTask(assigned_lane_id_);
  }

  return is_end_of_life;
}

bool BlindSpotModule::setStopLineIdx(const int current_pose_closest, const double judge_line_dist,
                                     autoware_planning_msgs::PathWithLaneId& path, int& stop_line_idx,
                                     int& judge_line_idx) {
  // TEMP: return first assigned_lane_id point's index
  stop_line_idx = -1;
  for (size_t i = 0; i < path.points.size(); ++i) {
    for (const auto& id : path.points.at(i).lane_ids) {
      if (id == assigned_lane_id_) {
        stop_line_idx = i;
      }
      if (stop_line_idx != -1) break;
    }
    if (stop_line_idx != -1) break;
  }

  if (stop_line_idx == -1) {
    ROS_ERROR(
        "[BlindSpotModule::setStopLineIdx]: cannot set the stop line. something wrong. please "
        "check code. ");
    return false;  // cannot find stop line.
  }

  // TEMP: should use interpolation (points distance may be very long)
  double curr_dist = 0.0;
  double prev_dist = curr_dist;
  judge_line_idx = -1;

  for (size_t i = stop_line_idx; i > 0; --i) {
    const geometry_msgs::Pose p0 = path.points.at(i).point.pose;
    const geometry_msgs::Pose p1 = path.points.at(i - 1).point.pose;
    curr_dist += planning_utils::calcDist2d(p0, p1);
    if (curr_dist > judge_line_dist) {
      const double dl = std::max(curr_dist - prev_dist, 0.0001 /* avoid 0 divide */);
      const double w_p0 = (curr_dist - judge_line_dist) / dl;
      const double w_p1 = (judge_line_dist - prev_dist) / dl;
      autoware_planning_msgs::PathPointWithLaneId p = path.points.at(i);
      p.point.pose.position.x = w_p0 * p0.position.x + w_p1 * p1.position.x;
      p.point.pose.position.y = w_p0 * p0.position.y + w_p1 * p1.position.y;
      p.point.pose.position.z = w_p0 * p0.position.z + w_p1 * p1.position.z;
      tf2::Quaternion q0_tf, q1_tf;
      tf2::fromMsg(p0.orientation, q0_tf);
      tf2::fromMsg(p1.orientation, q1_tf);
      p.point.pose.orientation = tf2::toMsg(q0_tf.slerp(q1_tf, w_p1));
      auto itr = path.points.begin();
      itr += i;
      path.points.insert(itr, p);
      judge_line_idx = i;
      ++stop_line_idx;
      break;
    }
    prev_dist = curr_dist;
  }
  if (judge_line_idx == -1) {
    ROS_DEBUG(
        "[BlindSpotModule::setStopLineIdx]: cannot set the stop judgement line. path is too short, "
        "or "
        "the vehicle is already ahead of the stop line. stop_line_id = %d",
        stop_line_idx);
  }
  return true;
}

geometry_msgs::Pose BlindSpotModule::getAheadPose(const size_t start_idx, const double ahead_dist,
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

bool BlindSpotModule::setVelocityFrom(const size_t idx, const double vel,
                                      autoware_planning_msgs::PathWithLaneId& input) {
  for (size_t i = idx; i < input.points.size(); ++i) {
    input.points.at(i).point.twist.linear.x = std::min(vel, input.points.at(i).point.twist.linear.x);
  }
}

Polygon BlindSpotModule::convertToBoostGeometryPolygon(const std::vector<geometry_msgs::Point>& detection_area) {
  Polygon polygon;
  for (const auto& p : detection_area) {
    polygon.outer().push_back(bg::make<Point>(p.x, p.y));
  }
  polygon.outer().push_back(polygon.outer().front());
  return polygon;
}

bool BlindSpotModule::checkCollision(
    const autoware_planning_msgs::PathWithLaneId& path,
    const std::vector<std::vector<geometry_msgs::Point>>& detection_areas,
    const std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects_ptr, const double path_width,
    bool& is_collision) {
  /* generates side edge line */
  autoware_planning_msgs::PathWithLaneId path_r;  // right side edge line
  autoware_planning_msgs::PathWithLaneId path_l;  // left side edge line
  generateEdgeLine(path, path_width, path_r, path_l);

  /* check collision for each objects and lanelets area */
  is_collision = false;
  for (size_t i = 0; i < detection_areas.size(); ++i)  // for each objective lanelets
  {
    Polygon polygon = convertToBoostGeometryPolygon(detection_areas.at(i));

    for (size_t j = 0; j < objects_ptr->objects.size(); ++j)  // for each dynamic objects
    {
      Point point(objects_ptr->objects.at(j).state.pose_covariance.pose.position.x,
                  objects_ptr->objects.at(j).state.pose_covariance.pose.position.y);
      if (bg::within(point,
                     polygon))  // if the dynamic object is in the lanelet polygon, check collision
      {
        // ROS_INFO("lanelet_id: %lu, object_no: %lu, INSIDE POLYGON", i, j);
        if (checkPathCollision(path_r, objects_ptr->objects.at(j)) ||
            checkPathCollision(path_l, objects_ptr->objects.at(j))) {
          is_collision = true;
        }
      } else {
        // ROS_INFO("lanelet_id: %lu, object_no: %lu, out of polygon", i, j);
      }

      if (is_collision) break;
    }
    if (is_collision) break;
  }

  /* for debug */
  blind_spot_module_manager_->debugger_.publishPath(path_r, "path_right_edge", 0.5, 0.0, 0.5);
  blind_spot_module_manager_->debugger_.publishPath(path_l, "path_left_edge", 0.0, 0.5, 0.5);

  return true;
}

bool BlindSpotModule::checkPathCollision(const autoware_planning_msgs::PathWithLaneId& path,
                                         const autoware_perception_msgs::DynamicObject& object) {
  bool is_collision = false;

  bg::model::linestring<Point> bg_ego_path;
  for (const auto& p : path.points) {
    bg_ego_path.push_back(Point{p.point.pose.position.x, p.point.pose.position.y});
  }

  std::vector<bg::model::linestring<Point>> bg_object_path_arr;
  for (size_t i = 0; i < object.state.predicted_paths.size(); ++i) {
    bg::model::linestring<Point> bg_object_path;
    for (const auto& p : object.state.predicted_paths.at(i).path) {
      bg_object_path.push_back(Point{p.pose.pose.position.x, p.pose.pose.position.y});
    }
    bg_object_path_arr.push_back(bg_object_path);
  }

  for (size_t i = 0; i < object.state.predicted_paths.size(); ++i) {
    bool is_intersects = bg::intersects(bg_ego_path, bg_object_path_arr.at(i));
    is_collision = is_collision || is_intersects;
  }

  return is_collision;
}

bool BlindSpotModule::generateEdgeLine(const autoware_planning_msgs::PathWithLaneId& path, const double path_width,
                                       autoware_planning_msgs::PathWithLaneId& path_r,
                                       autoware_planning_msgs::PathWithLaneId& path_l) {
  path_r = path;
  path_l = path;
  for (int i = 0; i < path.points.size(); ++i) {
    const double yaw = tf2::getYaw(path.points.at(i).point.pose.orientation);
    path_r.points.at(i).point.pose.position.x += path_width * std::sin(yaw);
    path_r.points.at(i).point.pose.position.y -= path_width * std::cos(yaw);
    path_l.points.at(i).point.pose.position.x -= path_width * std::sin(yaw);
    path_l.points.at(i).point.pose.position.y += path_width * std::cos(yaw);
  }
}

void BlindSpotModule::StateMachine::setStateWithMarginTime(BlindSpotModule::State state) {
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
        // ROS_INFO("[BlindSpotModule::StateMachine::setStateWithMarginTime()]: timer counting...
        // (%3.3f < %3.3f)", duration, margin_time_);
      } else {
        // ROS_INFO("[BlindSpotModule::StateMachine::setStateWithMarginTime()]: state changed. STOP
        // -> GO (%3.3f > %3.3f)", duration, margin_time_);
      }
      return;
    }
  }

  ROS_ERROR(
      "[BlindSpotModule::StateMachine::setStateWithMarginTime()] : Unsuitable state. ignore "
      "request.");
  return;
}

void BlindSpotModule::StateMachine::setState(BlindSpotModule::State state) { state_ = state; }

void BlindSpotModule::StateMachine::setMarginTime(const double t) { margin_time_ = t; }

BlindSpotModule::State BlindSpotModule::StateMachine::getState() { return state_; }
/*
 * ========================= BlindSpot Module Manager =========================
 */
bool BlindSpotModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                            std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  /* get self pose */
  geometry_msgs::PoseStamped self_pose;
  if (!getCurrentSelfPose(self_pose)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[BlindSpotModuleManager::startCondition()] cannot get current self pose");
    return false;
  }

  /* get lanelet map */
  lanelet::LaneletMapConstPtr lanelet_map_ptr;               // objects info
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;  // route info
  if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[BlindSpotModuleManager::startCondition()] cannot get lanelet map");
    return false;
  }

  /* search blind_spot tag */
  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      const int lane_id = input.points.at(i).lane_ids.at(j);
      lanelet::ConstLanelet lanelet_ij = lanelet_map_ptr->laneletLayer.get(lane_id);  // get lanelet layer
      std::string turn_direction = lanelet_ij.attributeOr("turn_direction", "else");  // get turn_direction

      if (!isRunning(lane_id)) {
        // check blind_spot tag
        if (turn_direction.compare("right") == 0 || turn_direction.compare("left") == 0)  // no plan for straight
        {
          // blind_spot tag is found. set module.
          v_module_ptr.push_back(std::make_shared<BlindSpotModule>(lane_id, turn_direction, this));
          registerTask(lane_id);
        }
      }
    }
  }

  return true;
}

bool BlindSpotModuleManager::isRunning(const int lane_id) {
  for (const auto& id : registered_lane_ids_) {
    if (id == lane_id) return true;
  }
  return false;
}

void BlindSpotModuleManager::registerTask(const int lane_id) { registered_lane_ids_.push_back(lane_id); }

void BlindSpotModuleManager::unregisterTask(const int lane_id) {
  const auto itr = std::find(registered_lane_ids_.begin(), registered_lane_ids_.end(), lane_id);
  if (itr == registered_lane_ids_.end())
    ROS_ERROR(
        "[BlindSpotModuleManager::unregisterTask()] : cannot remove task (lane_id = %d,"
        " registered_lane_ids_.size() = %lu)",
        lane_id, registered_lane_ids_.size());
  registered_lane_ids_.erase(itr);
}

/*
 * ========================= BlindSpot Module Debugger =========================
 */
BlindSpotModuleDebugger::BlindSpotModuleDebugger() : nh_(""), pnh_("~") {
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/blind_spot", 20);
}

void BlindSpotModuleDebugger::publishDetectionArea(
    const std::vector<std::vector<geometry_msgs::Point>>& detection_areas, int mode, const std::string& ns) {
  ros::Time curr_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  for (size_t i = 0; i < detection_areas.size(); ++i) {
    std::vector<geometry_msgs::Point> detection_area = detection_areas.at(i);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = curr_time;

    marker.ns = ns + "_" + std::to_string(i);
    marker.id = i;
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.99;  // Don't forget to set the alpha!
    if (mode == 0) {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
    } else {
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
    }
    for (size_t j = 0; j < detection_area.size(); ++j) {
      geometry_msgs::Point point;
      point.x = detection_area[j].x;
      point.y = detection_area[j].y;
      point.z = detection_area[j].z;
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }
  debug_viz_pub_.publish(msg);
}

void BlindSpotModuleDebugger::publishPath(const autoware_planning_msgs::PathWithLaneId& path, const std::string& ns,
                                          double r, double g, double b) {
  ros::Time curr_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = curr_time;
  marker.ns = ns;

  for (int i = 0; i < path.points.size(); ++i) {
    marker.id = i;
    marker.lifetime = ros::Duration(0.3);
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = path.points.at(i).point.pose;
    marker.scale.x = 0.5;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    msg.markers.push_back(marker);
  }

  debug_viz_pub_.publish(msg);
}

void BlindSpotModuleDebugger::publishPose(const geometry_msgs::Pose& pose, const std::string& ns, double r, double g,
                                          double b, int mode) {
  ros::Time curr_time = ros::Time::now();
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = curr_time;
  marker.ns = ns;

  marker.id = 0;
  marker.lifetime = ros::Duration(0.3);
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = pose;
  marker.scale.x = 0.5;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.color.a = 0.999;  // Don't forget to set the alpha!
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  msg.markers.push_back(marker);

  if (mode == 0)  // STOP
  {
    visualization_msgs::Marker marker_line;
    marker_line.header.frame_id = "map";
    marker_line.header.stamp = curr_time;
    marker_line.ns = ns + "_line";
    marker_line.id = 1;
    marker_line.lifetime = ros::Duration(0.3);
    marker_line.type = visualization_msgs::Marker::LINE_STRIP;
    marker_line.action = visualization_msgs::Marker::ADD;
    marker_line.pose.orientation.w = 1.0;
    marker_line.scale.x = 0.1;
    marker_line.color.a = 0.99;  // Don't forget to set the alpha!
    marker_line.color.r = r;
    marker_line.color.g = g;
    marker_line.color.b = b;

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

void BlindSpotModuleDebugger::publishGeofence(const geometry_msgs::Pose& pose, int32_t lane_id) {
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker marker_geofence{};
  marker_geofence.header.frame_id = "map";
  marker_geofence.header.stamp = ros::Time::now();
  marker_geofence.ns = "stop geofence";
  marker_geofence.id = lane_id;
  marker_geofence.lifetime = ros::Duration(0.5);
  marker_geofence.type = visualization_msgs::Marker::CUBE;
  marker_geofence.action = visualization_msgs::Marker::ADD;
  marker_geofence.pose = pose;
  marker_geofence.pose.position.z += 1.0;
  marker_geofence.scale.x = 0.1;
  marker_geofence.scale.y = 5.0;
  marker_geofence.scale.z = 2.0;
  marker_geofence.color.r = 1.0;
  marker_geofence.color.g = 0.0;
  marker_geofence.color.b = 0.0;
  marker_geofence.color.a = 0.5;
  msg.markers.push_back(marker_geofence);

  visualization_msgs::Marker marker_factor_text{};
  marker_factor_text.header.frame_id = "map";
  marker_factor_text.header.stamp = ros::Time::now();
  marker_factor_text.ns = "factor text";
  marker_factor_text.id = lane_id;
  marker_factor_text.lifetime = ros::Duration(0.5);
  marker_factor_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker_factor_text.action = visualization_msgs::Marker::ADD;
  marker_factor_text.pose = pose;
  marker_factor_text.pose.position.z += 2.0;
  marker_factor_text.scale.x = 0.0;
  marker_factor_text.scale.y = 0.0;
  marker_factor_text.scale.z = 1.0;
  marker_factor_text.color.r = 1.0;
  marker_factor_text.color.g = 1.0;
  marker_factor_text.color.b = 1.0;
  marker_factor_text.color.a = 0.999;
  marker_factor_text.text = "blind spot";
  msg.markers.push_back(marker_factor_text);

  debug_viz_pub_.publish(msg);
}

}  // namespace behavior_planning
#include <scene_module/blind_spot/scene.h>

#include "utilization/util.h"

#include <scene_module/blind_spot/manager.h>

namespace behavior_planning {

namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;

BlindSpotModule::BlindSpotModule(const int lane_id, const std::string& turn_direction,
                                 BlindSpotModuleManager* blind_spot_module_manager)
    : assigned_lane_id_(lane_id),
      turn_direction_(turn_direction),
      blind_spot_module_manager_(blind_spot_module_manager) {
  judge_line_dist_ = 1.5;             // [m]
  state_machine_.setMarginTime(2.0);  // [sec]
  path_expand_width_ = 2.0;
  show_debug_info_ = false;
}

bool BlindSpotModule::run(const autoware_planning_msgs::PathWithLaneId& input,
                          autoware_planning_msgs::PathWithLaneId& output) {
  output = input;
  blind_spot_module_manager_->debugger_.publishPath(output, "path_raw", 0.0, 1.0, 1.0);

  State current_state = state_machine_.getState();
  ROS_DEBUG_COND(show_debug_info_, "[BlindSpotModule]: run: state_machine_.getState() = %d", (int)current_state);

  /* get current pose */
  geometry_msgs::PoseStamped current_pose = planner_data_->current_pose;

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
        getAheadPose(stop_line_idx_, planner_data_->base_link2front, output), assigned_lane_id_);
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
  const auto objects_ptr = planner_data_->dynamic_objects;

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

bool BlindSpotModule::checkCollision(const autoware_planning_msgs::PathWithLaneId& path,
                                     const std::vector<std::vector<geometry_msgs::Point>>& detection_areas,
                                     const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr,
                                     const double path_width, bool& is_collision) {
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

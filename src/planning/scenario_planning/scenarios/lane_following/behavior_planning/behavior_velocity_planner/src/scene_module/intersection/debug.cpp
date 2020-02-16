#include <scene_module/intersection/scene.h>

#include "utilization/marker_helper.h"
#include "utilization/util.h"

namespace {

using State = IntersectionModule::State;

visualization_msgs::MarkerArray createLaneletsAreaMarkerArray(const std::vector<lanelet::ConstLanelet>& lanelets,
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

  return msg;
}

visualization_msgs::MarkerArray createPathMarkerArray(const autoware_planning_msgs::PathWithLaneId& path,
                                                      const std::string& ns, const double r, const double g,
                                                      const double b) {
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

  return msg;
}

visualization_msgs::MarkerArray createGeofenceMarkerArray(const geometry_msgs::Pose& pose, int32_t lane_id) {
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
  marker_geofence.scale = createMarkerScale(0.1, 5.0, 2.0);
  marker_geofence.color = createMarkerColor(1.0, 0.0, 0.0, 0.5);
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
  marker_factor_text.scale = createMarkerScale(0.0, 0.0, 1.0);
  marker_factor_text.color = createMarkerColor(1.0, 1.0, 1.0, 0.999);
  marker_factor_text.text = "intersection";
  msg.markers.push_back(marker_factor_text);

  return msg;
}

visualization_msgs::MarkerArray createPoseMarkerArray(const geometry_msgs::Pose& pose, const State& state,
                                                      const std::string& ns, const double r, const double g,
                                                      const double b) {
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

  if (state == State::STOP) {
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

  return msg;
}

}  // namespace

visualization_msgs::MarkerArray IntersectionModule::createDebugMarkerArray() {
  visualization_msgs::MarkerArray debug_marker_array;

  const auto state = state_machine_.getState();

  appendMarkerArray(createPathMarkerArray(debug_data_.path_raw, "path_raw", 0.0, 1.0, 1.0), &debug_marker_array);

  appendMarkerArray(createGeofenceMarkerArray(debug_data_.geofence_pose, lane_id_), &debug_marker_array);

  appendMarkerArray(createPoseMarkerArray(debug_data_.stop_point_pose, state, "stop_point_pose", 1.0, 0.0, 0.0),
                    &debug_marker_array);

  appendMarkerArray(createPoseMarkerArray(debug_data_.judge_point_pose, state, "judge_point_pose", 1.0, 1.0, 0.5),
                    &debug_marker_array);

  appendMarkerArray(createPathMarkerArray(debug_data_.path_with_judgeline, "path_with_judgeline", 0.0, 0.5, 1.0),
                    &debug_marker_array);

  appendMarkerArray(
      createLaneletsAreaMarkerArray(debug_data_.intersection_detection_lanelets, "intersection_detection_lanelets"),
      &debug_marker_array);

  appendMarkerArray(createPathMarkerArray(debug_data_.path_right_edge, "path_right_edge", 0.5, 0.0, 0.5),
                    &debug_marker_array);

  appendMarkerArray(createPathMarkerArray(debug_data_.path_left_edge, "path_left_edge", 0.0, 0.5, 0.5),
                    &debug_marker_array);

  return debug_marker_array;
}

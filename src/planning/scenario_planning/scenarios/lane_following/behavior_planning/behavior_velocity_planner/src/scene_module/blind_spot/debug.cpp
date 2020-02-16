#include <scene_module/blind_spot/debug.h>

#include "utilization/marker_helper.h"
#include "utilization/util.h"

BlindSpotModuleDebugger::BlindSpotModuleDebugger() : nh_(""), pnh_("~") {
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/blind_spot", 20);
}

void BlindSpotModuleDebugger::publishDetectionArea(
    const std::vector<std::vector<geometry_msgs::Point>>& detection_areas, int mode, const std::string& ns) {
  visualization_msgs::MarkerArray msg;

  auto marker = createDefaultMarker("map", ns.c_str(), 0, visualization_msgs::Marker::LINE_STRIP,
                                    createMarkerColor(0.0, 0.0, 0.0, 0.999));
  marker.lifetime = ros::Duration(0.3);
  marker.pose.orientation = createMarkerOrientation(0.0, 0.0, 0.0, 1.0);
  marker.scale = createMarkerScale(0.1, 0.0, 0.0);

  for (size_t i = 0; i < detection_areas.size(); ++i) {
    visualization_msgs::Marker marker;

    marker.ns = ns + "_" + std::to_string(i);
    marker.id = i;

    if (mode == 0) {
      marker.color = createMarkerColor(1.0, 0.0, 0.0, 0.999);
    } else {
      marker.color = createMarkerColor(0.0, 1.0, 1.0, 0.999);
    }

    // Add each vertex
    for (const auto& area_point : detection_areas.at(i)) {
      marker.points.push_back(area_point);
    }

    // Close polygon
    marker.points.push_back(marker.points.front());

    msg.markers.push_back(marker);
  }

  debug_viz_pub_.publish(msg);
}

void BlindSpotModuleDebugger::publishPath(const autoware_planning_msgs::PathWithLaneId& path, const std::string& ns,
                                          double r, double g, double b) {
  visualization_msgs::MarkerArray msg;

  auto marker =
      createDefaultMarker("map", ns.c_str(), 0, visualization_msgs::Marker::ARROW, createMarkerColor(r, g, b, 0.999));
  marker.lifetime = ros::Duration(0.3);
  marker.scale = createMarkerScale(0.5, 0.3, 0.3);

  for (int i = 0; i < path.points.size(); ++i) {
    marker.id = i;
    marker.pose = path.points.at(i).point.pose;
    msg.markers.push_back(marker);
  }

  debug_viz_pub_.publish(msg);
}

void BlindSpotModuleDebugger::publishPose(const geometry_msgs::Pose& pose, const std::string& ns, double r, double g,
                                          double b, int mode) {
  visualization_msgs::MarkerArray msg;

  auto marker =
      createDefaultMarker("map", ns.c_str(), 0, visualization_msgs::Marker::ARROW, createMarkerColor(r, g, b, 0.999));
  marker.id = 0;
  marker.lifetime = ros::Duration(0.3);
  marker.pose = pose;
  marker.scale = createMarkerScale(0.5, 0.3, 0.3);
  msg.markers.push_back(marker);

  // STOP
  if (mode == 0) {
    auto marker_line = createDefaultMarker("map", (ns + "_line").c_str(), 0, visualization_msgs::Marker::LINE_STRIP,
                                           createMarkerColor(r, g, b, 0.999));
    marker_line.id = 1;
    marker_line.lifetime = ros::Duration(0.3);
    marker_line.scale = createMarkerScale(0.1, 0.0, 0.0);

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

#include <scene_module/intersection/debug.h>

#include <tf2/utils.h>

#include "utilization/marker_helper.h"

namespace behavior_planning {

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

void IntersectionModuleDebugger::publishGeofence(const geometry_msgs::Pose& pose, int32_t lane_id) {
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

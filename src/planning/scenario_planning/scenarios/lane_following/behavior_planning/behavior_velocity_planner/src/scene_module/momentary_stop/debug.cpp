#include <scene_module/momentary_stop/debug.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

MomentaryStopDebugMarkersManager::MomentaryStopDebugMarkersManager() : nh_(), pnh_("~") {
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/momentary_stop", 1);
}

void MomentaryStopDebugMarkersManager::pushStopPose(const geometry_msgs::Pose& pose) {
  stop_poses_.push_back(geometry_msgs::Pose(pose));
}

void MomentaryStopDebugMarkersManager::publish() {
  visualization_msgs::MarkerArray msg;
  ros::Time current_time = ros::Time::now();
  double base_link2front = 0.0;  // TODO: fix
  tf2::Transform tf_base_link2front(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(base_link2front, 0.0, 0.0));

  // Stop Geofence
  for (size_t j = 0; j < stop_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "stop geofence";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(stop_poses_.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }
  // Facto Text
  for (size_t j = 0; j < stop_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "factor text";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(stop_poses_.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 2.0;
    marker.scale.x = 0.0;
    marker.scale.y = 0.0;
    marker.scale.z = 1.0;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.text = "stop line";
    msg.markers.push_back(marker);
  }
  debug_viz_pub_.publish(msg);
  stop_poses_.clear();

  return;
}

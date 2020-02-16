#include <scene_module/traffic_light/debug.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace behavior_planning {
// TrafficLightDebugMarkersManager::TrafficLightDebugMarkersManager() : nh_(), pnh_("~"),
// tf_listener_(tf_buffer_)
TrafficLightDebugMarkersManager::TrafficLightDebugMarkersManager() : nh_(), pnh_("~") {
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/traffic_light", 1);
}

void TrafficLightDebugMarkersManager::pushTrafficLightState(
    const std::shared_ptr<lanelet::TrafficLight const>& traffic_light,
    const autoware_traffic_light_msgs::TrafficLightState& state) {
  tl_state_.push_back(std::make_tuple(traffic_light, state));
}

void TrafficLightDebugMarkersManager::pushStopPose(const geometry_msgs::Pose& pose) {
  stop_poses_.push_back(geometry_msgs::Pose(pose));
}

void TrafficLightDebugMarkersManager::pushJudgePose(const geometry_msgs::Pose& pose) {
  judge_poses_.push_back(geometry_msgs::Pose(pose));
}

#if 0
void MapBasedDetector::publishVisibleTrafficLights(const geometry_msgs::PoseStamped camera_pose_stamped,
                                                   const std::vector<lanelet::ConstLineString3d> &visible_traffic_lights,
                                                   const ros::Publisher &pub)
{
  visualization_msgs::MarkerArray output_msg;
  for (const auto &traffic_light : visible_traffic_lights)
  {
    const auto &tl_left_down_point = traffic_light.front();
    const auto &tl_right_down_point = traffic_light.back();
    const double tl_height = traffic_light.attributeOr("height", 0.0);
    const int id = traffic_light.id();

    geometry_msgs::Point tl_central_point;
    tl_central_point.x = (tl_right_down_point.x() + tl_left_down_point.x()) / 2.0;
    tl_central_point.y = (tl_right_down_point.y() + tl_left_down_point.y()) / 2.0;
    tl_central_point.z = (tl_right_down_point.z() + tl_left_down_point.z() + tl_height) / 2.0;

    visualization_msgs::Marker marker;

    tf2::Transform tf_map2camera(tf2::Quaternion(camera_pose_stamped.pose.orientation.x, camera_pose_stamped.pose.orientation.y, camera_pose_stamped.pose.orientation.z, camera_pose_stamped.pose.orientation.w),
                                 tf2::Vector3(camera_pose_stamped.pose.position.x, camera_pose_stamped.pose.position.y, camera_pose_stamped.pose.position.z));
    tf2::Transform tf_map2tl(tf2::Quaternion(0, 0, 0, 1),
                             tf2::Vector3(tl_central_point.x, tl_central_point.y, tl_central_point.z));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

    marker.header = camera_pose_stamped.header;
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_LIST;
    marker.ns = std::string("beam");
    marker.scale.x = 0.05;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    geometry_msgs::Point point;
    point.x = 0.0;
    point.y = 0.0;
    point.z = 0.0;
    marker.points.push_back(point);
    point.x = tf_camera2tl.getOrigin().x();
    point.y = tf_camera2tl.getOrigin().y();
    point.z = tf_camera2tl.getOrigin().z();
    marker.points.push_back(point);

    marker.lifetime = ros::Duration(0.2);
    marker.color.a = 0.999; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    output_msg.markers.push_back(marker);
  }
  pub.publish(output_msg);

  return;
}
#endif

void TrafficLightDebugMarkersManager::publish() {
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
    marker.text = "traffic light";
    msg.markers.push_back(marker);
  }
  // Judge Geofence
  for (size_t j = 0; j < judge_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "judge geofence";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(judge_poses_.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }
  // Facto Text
  for (size_t j = 0; j < judge_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "judge factor text";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(judge_poses_.at(j), tf_map2base_link);
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
    marker.text = "traffic light (judge)";
    msg.markers.push_back(marker);
  }

  debug_viz_pub_.publish(msg);
  tl_state_.clear();
  stop_poses_.clear();
  judge_poses_.clear();

  return;
}

}  // namespace behavior_planning

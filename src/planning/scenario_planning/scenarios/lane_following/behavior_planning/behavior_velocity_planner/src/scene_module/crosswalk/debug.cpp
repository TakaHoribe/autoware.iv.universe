#include <scene_module/crosswalk/debug.hpp>

#include <visualization_msgs/MarkerArray.h>

namespace behavior_planning {

CrosswalkDebugMarkersManager::CrosswalkDebugMarkersManager() : nh_(), pnh_("~") {
  debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/crosswalk", 1);
}

void CrosswalkDebugMarkersManager::pushCollisionLine(const std::vector<Eigen::Vector3d>& line) {
  collision_lines_.push_back(std::vector<Eigen::Vector3d>(line));
}

void CrosswalkDebugMarkersManager::pushCollisionLine(const std::vector<Eigen::Vector2d>& line) {
  std::vector<Eigen::Vector3d> line3d;
  for (const auto& point : line) {
    Eigen::Vector3d point3d;
    point3d << point.x(), point.y(), 0.0;
    line3d.push_back(point3d);
  }
  pushCollisionLine(line3d);
}

void CrosswalkDebugMarkersManager::pushCollisionPoint(const Eigen::Vector3d& point) {
  collision_points_.push_back(Eigen::Vector3d(point));
}

void CrosswalkDebugMarkersManager::pushCollisionPoint(const Eigen::Vector2d& point) {
  Eigen::Vector3d point3d;
  point3d << point.x(), point.y(), 0.0;
  pushCollisionPoint(point3d);
}

void CrosswalkDebugMarkersManager::pushStopPose(const geometry_msgs::Pose& pose) {
  stop_poses_.push_back(geometry_msgs::Pose(pose));
}

void CrosswalkDebugMarkersManager::pushSlowPose(const geometry_msgs::Pose& pose) {
  slow_poses_.push_back(geometry_msgs::Pose(pose));
}

void CrosswalkDebugMarkersManager::pushCrosswalkPolygon(const std::vector<Eigen::Vector3d>& polygon) {
  crosswalk_polygons_.push_back(std::vector<Eigen::Vector3d>(polygon));
}

void CrosswalkDebugMarkersManager::pushCrosswalkPolygon(const std::vector<Eigen::Vector2d>& polygon) {
  std::vector<Eigen::Vector3d> polygon3d;
  for (const auto& point : polygon) {
    Eigen::Vector3d point3d;
    point3d << point.x(), point.y(), 0;
    polygon3d.push_back(point3d);
  }
  pushCrosswalkPolygon(polygon3d);
}

void CrosswalkDebugMarkersManager::pushStopPolygon(const std::vector<Eigen::Vector3d>& polygon) {
  stop_polygons_.push_back(std::vector<Eigen::Vector3d>(polygon));
}

void CrosswalkDebugMarkersManager::pushStopPolygon(const std::vector<Eigen::Vector2d>& polygon) {
  std::vector<Eigen::Vector3d> polygon3d;
  for (const auto& point : polygon) {
    Eigen::Vector3d point3d;
    point3d << point.x(), point.y(), 0;
    polygon3d.push_back(point3d);
  }
  pushStopPolygon(polygon3d);
}

void CrosswalkDebugMarkersManager::pushSlowPolygon(const std::vector<Eigen::Vector3d>& polygon) {
  slow_polygons_.push_back(std::vector<Eigen::Vector3d>(polygon));
}

void CrosswalkDebugMarkersManager::pushSlowPolygon(const std::vector<Eigen::Vector2d>& polygon) {
  std::vector<Eigen::Vector3d> polygon3d;
  for (const auto& point : polygon) {
    Eigen::Vector3d point3d;
    point3d << point.x(), point.y(), 0;
    polygon3d.push_back(point3d);
  }
  pushSlowPolygon(polygon3d);
}

void CrosswalkDebugMarkersManager::publish() {
  visualization_msgs::MarkerArray msg;
  ros::Time current_time = ros::Time::now();
  double base_link2front = 0.0;  // TODO: fix
  tf2::Transform tf_base_link2front(tf2::Quaternion(0.0, 0.0, 0.0, 1.0), tf2::Vector3(base_link2front, 0.0, 0.0));

  // Crosswalk polygons
  for (size_t i = 0; i < crosswalk_polygons_.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = crosswalk_polygons_.at(i);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = "crosswalk polygon line";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);

    marker.ns = "crosswalk polygon point";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Collision line
  for (size_t i = 0; i < collision_lines_.size(); ++i) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "collision line";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < collision_lines_.at(i).size(); ++j) {
      geometry_msgs::Point point;
      point.x = collision_lines_.at(i).at(j).x();
      point.y = collision_lines_.at(i).at(j).y();
      point.z = collision_lines_.at(i).at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Collision point
  if (!collision_points_.empty()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "collision point";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < collision_points_.size(); ++j) {
      geometry_msgs::Point point;
      point.x = collision_points_.at(j).x();
      point.y = collision_points_.at(j).y();
      point.z = collision_points_.at(j).z();
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Slow polygon
  for (size_t i = 0; i < slow_polygons_.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = slow_polygons_.at(i);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = "slow polygon line";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Slow point
  if (!slow_poses_.empty()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "slow point";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < slow_poses_.size(); ++j) {
      geometry_msgs::Point point;
      point.x = slow_poses_.at(j).position.x;
      point.y = slow_poses_.at(j).position.y;
      point.z = slow_poses_.at(j).position.z;
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

  // Stop polygon
  for (size_t i = 0; i < stop_polygons_.size(); ++i) {
    std::vector<Eigen::Vector3d> polygon = stop_polygons_.at(i);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;

    marker.ns = "stop polygon line";
    marker.id = i;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < polygon.size(); ++j) {
      geometry_msgs::Point point;
      point.x = polygon.at(j).x();
      point.y = polygon.at(j).y();
      point.z = polygon.at(j).z();
      marker.points.push_back(point);
    }
    marker.points.push_back(marker.points.front());
    msg.markers.push_back(marker);
  }

  // Stop point
  if (!stop_poses_.empty()) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "stop point";
    marker.id = 0;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 0;
    marker.pose.position.y = 0;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.color.a = 0.999;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    for (size_t j = 0; j < stop_poses_.size(); ++j) {
      geometry_msgs::Point point;
      point.x = stop_poses_.at(j).position.x;
      point.y = stop_poses_.at(j).position.y;
      point.z = stop_poses_.at(j).position.z;
      marker.points.push_back(point);
    }
    msg.markers.push_back(marker);
  }

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
  // Factor Text
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
    marker.text = "crosswalk";
    msg.markers.push_back(marker);
  }

  // Slow Geofence
  for (size_t j = 0; j < slow_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "slow geofence";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(slow_poses_.at(j), tf_map2base_link);
    tf2::Transform tf_map2front = tf_map2base_link * tf_base_link2front;
    tf2::toMsg(tf_map2front, marker.pose);
    marker.pose.position.z += 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 5.0;
    marker.scale.z = 2.0;
    marker.color.a = 0.5;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    msg.markers.push_back(marker);
  }
  // Slow Factor Text
  for (size_t j = 0; j < slow_poses_.size(); ++j) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = current_time;
    marker.ns = "slow factor text";
    marker.id = j;
    marker.lifetime = ros::Duration(0.5);
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;
    tf2::Transform tf_map2base_link;
    tf2::fromMsg(slow_poses_.at(j), tf_map2base_link);
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
    marker.text = "crosswalk";
    msg.markers.push_back(marker);
  }

  debug_viz_pub_.publish(msg);
  collision_points_.clear();
  stop_poses_.clear();
  slow_poses_.clear();
  collision_lines_.clear();
  crosswalk_polygons_.clear();
  slow_polygons_.clear();
  stop_polygons_.clear();

  return;
}

}  // namespace behavior_planning

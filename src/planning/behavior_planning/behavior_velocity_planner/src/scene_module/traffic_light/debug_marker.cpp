#include <scene_module/traffic_light/debug_marker.hpp>
#include <behavior_velocity_planner/api.hpp>

namespace behavior_planning
{
// TrafficLightDebugMarkersManager::TrafficLightDebugMarkersManager() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
TrafficLightDebugMarkersManager::TrafficLightDebugMarkersManager() : nh_(), pnh_("~")
{
    debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("traffic_light/output/debug/marker", 1);
}

void TrafficLightDebugMarkersManager::pushTrafficLightState(const std::shared_ptr<lanelet::TrafficLight const> &traffic_light,
                                                            const autoware_traffic_light_msgs::TrafficLightState &state)
{
    tl_state_.push_back(std::make_tuple(traffic_light, state));
}


void TrafficLightDebugMarkersManager::pushStopPose(const geometry_msgs::Pose &pose)
{
    stop_poses_.push_back(geometry_msgs::Pose(pose));
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
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    output_msg.markers.push_back(marker);
  }
  pub.publish(output_msg);

  return;
}
#endif

void TrafficLightDebugMarkersManager::publish()
{
    visualization_msgs::MarkerArray msg;
    ros::Time current_time = ros::Time::now();

#if 0
    const int lanelet_tl_ptr = 0;
    const int autoware_tl_state = 1;
    // Traffic Light States
    for (size_t i = 0; i < tl_state_.size(); ++i)
    {
        const std::tuple<std::shared_ptr<lanelet::TrafficLight const>,
                         autoware_traffic_light_msgs::TrafficLightState> &tl_state = tl_state_.at(i);
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = current_time;

        marker.ns = "text";
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < polygon.size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = polygon.at(j).x();
            point.y = polygon.at(j).y();
            point.z = polygon.at(j).z();
            marker.points.push_back(point);
        }
        marker.points.push_back(marker.points.front());
        msg.markers.push_back(marker);

    // Crosswalk polygons
    for (size_t i = 0; i < crosswalk_polygons_.size(); ++i)
    {
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < polygon.size(); ++j)
        {
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < polygon.size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = polygon.at(j).x();
            point.y = polygon.at(j).y();
            point.z = polygon.at(j).z();
            marker.points.push_back(point);
        }
        msg.markers.push_back(marker);
    }

    // Collision line
    for (size_t i = 0; i < collision_lines_.size(); ++i)
    {
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < collision_lines_.at(i).size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = collision_lines_.at(i).at(j).x();
            point.y = collision_lines_.at(i).at(j).y();
            point.z = collision_lines_.at(i).at(j).z();
            marker.points.push_back(point);
        }
        msg.markers.push_back(marker);
    }

    // Collision point
    if (!collision_points_.empty())
    {
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < collision_points_.size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = collision_points_.at(j).x();
            point.y = collision_points_.at(j).y();
            point.z = collision_points_.at(j).z();
            marker.points.push_back(point);
        }
        msg.markers.push_back(marker);
    }

    // Slow polygon
    for (size_t i = 0; i < slow_polygons_.size(); ++i)
    {
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        for (size_t j = 0; j < polygon.size(); ++j)
        {
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
    if (!slow_poses_.empty())
    {
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        for (size_t j = 0; j < slow_poses_.size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = slow_poses_.at(j).position.x;
            point.y = slow_poses_.at(j).position.y;
            point.z = slow_poses_.at(j).position.z;
            marker.points.push_back(point);
        }
        msg.markers.push_back(marker);
    }

    // Stop polygon
    for (size_t i = 0; i < stop_polygons_.size(); ++i)
    {
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < polygon.size(); ++j)
        {
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
    if (!stop_poses_.empty())
    {
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
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        for (size_t j = 0; j < stop_poses_.size(); ++j)
        {
            geometry_msgs::Point point;
            point.x = stop_poses_.at(j).position.x;
            point.y = stop_poses_.at(j).position.y;
            point.z = stop_poses_.at(j).position.z;
            marker.points.push_back(point);
        }
        msg.markers.push_back(marker);
    }


#endif
    // Geofence Stop
    for (size_t j = 0; j < stop_poses_.size(); ++j)
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = current_time;
        marker.ns = "geofence";
        marker.id = j;
        marker.lifetime = ros::Duration(0.5);
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = stop_poses_.at(j).position.x;
        marker.pose.position.y = stop_poses_.at(j).position.y;
        marker.pose.position.z = stop_poses_.at(j).position.z+1.0;
        marker.pose.orientation.x = stop_poses_.at(j).orientation.x;
        marker.pose.orientation.y = stop_poses_.at(j).orientation.y;
        marker.pose.orientation.z = stop_poses_.at(j).orientation.z;
        marker.pose.orientation.w = stop_poses_.at(j).orientation.w;
        marker.scale.x = 0.1;
        marker.scale.y = 5.0;
        marker.scale.z = 2.0;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        double base_link2front;
        getBaselink2FrontLength(base_link2front);
        geometry_msgs::Point point;
        point.x = base_link2front;
        point.y = 0;
        point.z = 0;
        marker.points.push_back(point);
        msg.markers.push_back(marker);
    }

    debug_viz_pub_.publish(msg);
    tl_state_.clear();
    stop_poses_.clear();

    return;
}

}
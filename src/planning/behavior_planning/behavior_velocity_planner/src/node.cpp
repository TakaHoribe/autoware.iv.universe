#include <behavior_velocity_planner/node.hpp>
#include <lanelet2_routing/Route.h>
#include <lanelet2_extension/utility/message_conversion.h>
// #include <lanelet2_extension/utility/query.h>
// #include <lanelet2_extension/visualization/visualization.h>
#include <visualization_msgs/MarkerArray.h>
#include <path_utils/utilization.hpp>
namespace behavior_planning
{
BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode() : nh_(), pnh_("~")
{
    path_with_lane_id_sub_ = pnh_.subscribe("input/path_with_lane_id", 1, &BehaviorVelocityPlannerNode::pathWithLaneIdCallback, this);
    perception_sub_ = pnh_.subscribe("input/perception", 1, &SingletonDataManager::perceptionCallback, &SingletonDataManager::getInstance());
    pointcloud_sub_ = pnh_.subscribe("input/pointcloud", 1, &SingletonDataManager::pointcloudCallback, &SingletonDataManager::getInstance());
    map_sub_ = pnh_.subscribe("input/lanelet_map_bin", 10, &SingletonDataManager::mapCallback, &SingletonDataManager::getInstance());
    path_pub_ = pnh_.advertise<autoware_planning_msgs::Path>("output/path", 1);
    debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/path", 1);
    pnh_.param("foward_path_length", foward_path_length_, 1000.0);
    pnh_.param("backward_path_length", backward_path_length_, 5.0);
    planner_manager_ptr_ = std::make_shared<BehaviorVelocityPlannerManager>();
};

void BehaviorVelocityPlannerNode::pathWithLaneIdCallback(const autoware_planning_msgs::PathWithLaneId &input_path_msg)
{
    // if (path_pub_.getNumSubscribers() < 1)
    autoware_planning_msgs::Path veloctiy_planed_path;
    if (planner_manager_ptr_->callback(input_path_msg, veloctiy_planed_path))
    {
        autoware_planning_msgs::Path filtered_path;
        filterPath(veloctiy_planed_path, filtered_path);
        autoware_planning_msgs::Path output_path_msg;
        interporatePath(filtered_path, foward_path_length_, output_path_msg);
        output_path_msg.header.frame_id = "map";
        output_path_msg.header.stamp = ros::Time::now();
        path_pub_.publish(output_path_msg);
        publishDebugMarker(output_path_msg, debug_viz_pub_);
    }

    return;
};

void BehaviorVelocityPlannerNode::publishDebugMarker(const autoware_planning_msgs::Path &path, const ros::Publisher &pub)
{
    if (pub.getNumSubscribers() < 1)
        return;
    visualization_msgs::MarkerArray output_msg;
    for (size_t i = 0; i < path.points.size(); ++i)
    {
        visualization_msgs::Marker marker;
        marker.header = path.header;
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.pose = path.points.at(i).pose;
        marker.scale.y = marker.scale.z = 0.05;
        marker.scale.x = 0.25;
        marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(0.5);
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        output_msg.markers.push_back(marker);
    }
    pub.publish(output_msg);
}
} // namespace behavior_planning

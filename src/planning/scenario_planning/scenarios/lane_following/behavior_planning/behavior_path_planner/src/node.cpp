#include <behavior_path_planner/node.hpp>
#include <lanelet2_routing/Route.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
// #include <lanelet2_extension/utility/query.h>
// #include <lanelet2_extension/visualization/visualization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <goal_path_refiner/goal_path_refiner.hpp>

namespace behavior_planning
{
BehaviorPathPlannerNode::BehaviorPathPlannerNode() : nh_(), pnh_("~")
{
    timer_ = nh_.createTimer(ros::Duration(0.1), &BehaviorPathPlannerNode::timerCallback, this);
    route_sub_ = pnh_.subscribe("input/route", 1, &BehaviorPathPlannerNode::routeCallback, this);
    perception_sub_ = pnh_.subscribe("input/perception", 1, &BehaviorPathPlannerNode::perceptionCallback, this);
    pointcloud_sub_ = pnh_.subscribe("input/pointcloud", 1, &BehaviorPathPlannerNode::pointcloudCallback, this);
    map_sub_ = pnh_.subscribe("input/lanelet_map_bin", 10, &BehaviorPathPlannerNode::mapCallback, this);
    path_with_lane_id_pub_ = pnh_.advertise<autoware_planning_msgs::PathWithLaneId>("output/path_with_lane_id", 1);
    debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/path", 1);
    pnh_.param("foward_path_length", foward_path_length_, 1000.0);
    pnh_.param("backward_path_length", backward_path_length_, 5.0);
};

void BehaviorPathPlannerNode::timerCallback(const ros::TimerEvent &e)
{
    // if (path_pub_.getNumSubscribers() < 1)
    //     return;
    if (route_ptr_ == nullptr)
        return;
    //   if(route_ptr_ == nullptr || perception_ptr_ == nullptr || pointcloud_ptr_ == nullptr)
    //     return;

    autoware_planning_msgs::PathWithLaneId output_path_with_lane_id_msg;
    if (callback(*route_ptr_, *perception_ptr_, *pointcloud_ptr_, output_path_with_lane_id_msg))
    {
        output_path_with_lane_id_msg.header.frame_id = "map";
        output_path_with_lane_id_msg.header.stamp = ros::Time::now();
        path_with_lane_id_pub_.publish(output_path_with_lane_id_msg);
        publishDebugMarker(output_path_with_lane_id_msg, debug_viz_pub_);
    }
}

void BehaviorPathPlannerNode::routeCallback(const autoware_planning_msgs::Route &input_route_msg)
{
    if (input_route_msg.route_sections.empty())
        ROS_WARN("route is empty");
    route_ptr_ = std::make_shared<autoware_planning_msgs::Route>(input_route_msg);
    cache_path_manager_.reset();
    return;
};

void BehaviorPathPlannerNode::perceptionCallback(const autoware_perception_msgs::DynamicObjectArray &input_perception_msg)
{
    perception_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(input_perception_msg);

    return;
};

void BehaviorPathPlannerNode::pointcloudCallback(const sensor_msgs::PointCloud2 &input_pointcloud_msg)
{
    pointcloud_ptr_ = std::make_shared<sensor_msgs::PointCloud2>(input_pointcloud_msg);

    return;
};

void BehaviorPathPlannerNode::mapCallback(const autoware_lanelet2_msgs::MapBin &input_map_msg)
{
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(input_map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

bool BehaviorPathPlannerNode::callback(const autoware_planning_msgs::Route &input_route_msg,
                                       const autoware_perception_msgs::DynamicObjectArray &input_perception_msg,
                                       const sensor_msgs::PointCloud2 &input_pointcloud_msg,
                                       autoware_planning_msgs::PathWithLaneId &output_path_msg)
{
    autoware_planning_msgs::PathWithLaneId path;
    // generate
    for (const auto &route_section : input_route_msg.route_sections)
    {
        for (const auto &lane_id : route_section.lane_ids)
        {
            if (lane_id != route_section.preferred_lane_id)
            {
              continue;
            }
            lanelet::ConstLineString3d center_line = lanelet::utils::generateFineCenterline(lanelet_map_ptr_->laneletLayer.get(lane_id), 5.0);
            lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(lanelet_map_ptr_->laneletLayer.get(lane_id));
            for (size_t i = 0; i < center_line.size(); ++i)
            // for (const auto &center_point : center_line)
            {
                autoware_planning_msgs::PathPointWithLaneId path_point;
                path_point.point.pose.position.x = center_line[i].x();
                path_point.point.pose.position.y = center_line[i].y();
                path_point.point.pose.position.z = center_line[i].z();
                path_point.point.twist.linear.x = limit.speedLimit.value();
                path_point.lane_ids.push_back(lane_id);
                path.points.push_back(path_point);
            }
        }
    }

    // screening path
    autoware_planning_msgs::PathWithLaneId filtered_path;
    filterPath(path, filtered_path);

    // generate path for goal pose
    autoware_planning_msgs::PathWithLaneId goal_path;
    if (!GoalPathRefiner::getRefinedPath(7.0, 3.14 / 2.0, filtered_path, input_route_msg.goal_pose, goal_path))
    {
        ROS_ERROR("Cannot find goal position");
        return false;
    }

    output_path_msg.points = goal_path.points;
    return true;
}

void BehaviorPathPlannerNode::filterPath(const autoware_planning_msgs::PathWithLaneId &path, autoware_planning_msgs::PathWithLaneId &filtered_path)
{
    const double epsilon = 0.1;
    size_t latest_id = 0;
    for (size_t i = 0; i < path.points.size(); ++i)
    {
        double dist;
        if (i != 0)
        {
            const double x = path.points.at(i).point.pose.position.x - path.points.at(latest_id).point.pose.position.x;
            const double y = path.points.at(i).point.pose.position.y - path.points.at(latest_id).point.pose.position.y;
            dist = std::sqrt(x * x + y * y);
        }
        if (epsilon < dist || i == 0 /*init*/)
        {
            latest_id = i;
            filtered_path.points.push_back(path.points.at(latest_id));
        }else {
            filtered_path.points.back().lane_ids.push_back(path.points.at(i).lane_ids.at(0));
            filtered_path.points.back().point.twist.linear.x = std::min(filtered_path.points.back().point.twist.linear.x,
                                                                  path.points.at(i).point.twist.linear.x);
        }
    }
}

void BehaviorPathPlannerNode::publishDebugMarker(const autoware_planning_msgs::PathWithLaneId &path, const ros::Publisher &pub)
{
    if (pub.getNumSubscribers() < 1)
        return;
    visualization_msgs::MarkerArray output_msg;
    {
        visualization_msgs::Marker marker;
        marker.header = path.header;
        marker.ns = "points";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.scale.x = marker.scale.y = 0.1;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0.5);
        marker.color.a = 0.999; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        for (size_t i = 0; i < path.points.size(); ++i)
        {
            marker.points.push_back(path.points.at(i).point.pose.position);
        }
        output_msg.markers.push_back(marker);
    }

    {
        visualization_msgs::Marker marker;
        marker.header = path.header;
        marker.ns = "line";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.scale.x = 0.05;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = ros::Duration(0.5);
        marker.color.a = 0.999; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        for (size_t i = 0; i < path.points.size(); ++i)
        {
            marker.points.push_back(path.points.at(i).point.pose.position);
        }
        output_msg.markers.push_back(marker);
    }

    {
        for (size_t i = 0; i < path.points.size(); ++i)
        {
            visualization_msgs::Marker marker;
            marker.header = path.header;
            marker.ns = "lane_id";
            marker.id = i;
            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.scale.z = 0.25;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose = path.points.at(i).point.pose;
            std::string text;
            if (path.points.at(i).lane_ids.empty())
                continue;
            for (size_t j = 0; j < path.points.at(i).lane_ids.size(); ++j)
            {
                text = text + std::string(std::to_string(path.points.at(i).lane_ids.at(j))) + std::string(",");
            }
            text.pop_back();
            marker.text = text;
            marker.lifetime = ros::Duration(0.5);
            marker.color.a = 0.999; // Don't forget to set the alpha!
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            output_msg.markers.push_back(marker);
        }
    }

    pub.publish(output_msg);
}

SelfPoseLinstener::SelfPoseLinstener() : tf_listener_(tf_buffer_){};
bool SelfPoseLinstener::getSelfPose(geometry_msgs::Pose &self_pose, const std_msgs::Header &header)
{
    try
    {
        geometry_msgs::TransformStamped transform;
        transform = tf_buffer_.lookupTransform(header.frame_id, "base_link",
                                               header.stamp, ros::Duration(0.1));
        self_pose.position.x = transform.transform.translation.x;
        self_pose.position.y = transform.transform.translation.y;
        self_pose.position.z = transform.transform.translation.z;
        self_pose.orientation.x = transform.transform.rotation.x;
        self_pose.orientation.y = transform.transform.rotation.y;
        self_pose.orientation.z = transform.transform.rotation.z;
        self_pose.orientation.w = transform.transform.rotation.w;
        return true;
    }
    catch (tf2::TransformException &ex)
    {
        return false;
    }
}

} // namespace behavior_planning

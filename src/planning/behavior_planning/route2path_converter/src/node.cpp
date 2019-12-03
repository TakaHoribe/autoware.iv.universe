#include <route2path_converter/node.hpp>
#include <lanelet2_routing/Route.h>

#include <lanelet2_extension/utility/message_conversion.h>
// #include <lanelet2_extension/utility/query.h>
// #include <lanelet2_extension/visualization/visualization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <goal_path_refiner/goal_path_refiner.hpp>

namespace behavior_planning
{
Route2PathConverterNode::Route2PathConverterNode() : nh_(), pnh_("~")
{
    timer_ = nh_.createTimer(ros::Duration(0.1), &Route2PathConverterNode::timerCallback, this);
    route_sub_ = pnh_.subscribe("input/route", 1, &Route2PathConverterNode::routeCallback, this);
    perception_pub_ = pnh_.subscribe("input/perception", 1, &Route2PathConverterNode::perceptionCallback, this);
    pointcloud_sub_ = pnh_.subscribe("input/pointcloud", 1, &Route2PathConverterNode::pointcloudCallback, this);
    map_sub_ = pnh_.subscribe("input/lanelet_map_bin", 10, &Route2PathConverterNode::mapCallback, this);
    path_pub_ = pnh_.advertise<autoware_planning_msgs::Path>("output/path", 1);
};

void Route2PathConverterNode::timerCallback(const ros::TimerEvent &e)
{
    // if (path_pub_.getNumSubscribers() < 1)
    //     return;
    if (route_ptr_ == nullptr)
        return;
    //   if(route_ptr_ == nullptr || perception_ptr_ == nullptr || pointcloud_ptr_ == nullptr)
    //     return;
    autoware_planning_msgs::Path output_path_msg;

    if (callback(*route_ptr_, *perception_ptr_, *pointcloud_ptr_, output_path_msg))
    {
        output_path_msg.header.frame_id = "map";
        output_path_msg.header.stamp = ros::Time::now();
        path_pub_.publish(output_path_msg);
    }
}

void Route2PathConverterNode::routeCallback(const autoware_planning_msgs::Route &input_route_msg)
{
    if (input_route_msg.route_sections.empty())
        ROS_WARN("route is empty");
    route_ptr_ = std::make_shared<autoware_planning_msgs::Route>(input_route_msg);

    return;
};

void Route2PathConverterNode::perceptionCallback(const autoware_perception_msgs::DynamicObjectArray &input_perception_msg)
{
    perception_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(input_perception_msg);

    return;
};

void Route2PathConverterNode::pointcloudCallback(const sensor_msgs::PointCloud2 &input_pointcloud_msg)
{
    pointcloud_ptr_ = std::make_shared<sensor_msgs::PointCloud2>(input_pointcloud_msg);

    return;
};

void Route2PathConverterNode::mapCallback(const autoware_lanelet2_msgs::MapBin &input_map_msg)
{
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(input_map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

bool Route2PathConverterNode::callback(const autoware_planning_msgs::Route &input_route_msg,
                                       const autoware_perception_msgs::DynamicObjectArray &input_perception_msg,
                                       const sensor_msgs::PointCloud2 &input_pointcloud_msg,
                                       autoware_planning_msgs::Path &output_path_msg)
{
    // test
    lanelet::ConstLanelets route_lanelets;
    for (const auto &route_section : input_route_msg.route_sections)
    {
        for (const auto &lane_id : route_section.lane_ids)
        {
            lanelet::ConstLineString3d center_line = (lanelet_map_ptr_->laneletLayer.get(lane_id)).centerline();
            lanelet::traffic_rules::SpeedLimitInformation limit = traffic_rules_ptr_->speedLimit(lanelet_map_ptr_->laneletLayer.get(lane_id));
            for (const auto &center_point : center_line)
            {
                autoware_planning_msgs::PathPoint point;
                point.pose.position.x = center_point.x();
                point.pose.position.y = center_point.y();
                point.pose.position.z = center_point.z();
                point.twist.linear.x = limit.speedLimit.value();

                output_path_msg.points.push_back(point);

            }
        }
    }
    if (output_path_msg.points.size() < 3)
    {
        ROS_ERROR("minimum points size is 2");
        return false;
    }
     // orientation
    for (size_t i = 0; i < output_path_msg.points.size(); ++i)
    {
        tf2::Quaternion tf2_quaternion;
        double yaw = 0.0;
        if (i == 0)
        {
            yaw = std::atan2(output_path_msg.points.at(i + 1).pose.position.y - output_path_msg.points.at(i).pose.position.y,
                             output_path_msg.points.at(i + 1).pose.position.x - output_path_msg.points.at(i).pose.position.x);
        }
        else if (i + 1 == output_path_msg.points.size())
        {
            yaw = std::atan2(output_path_msg.points.at(i).pose.position.y - output_path_msg.points.at(i - 1).pose.position.y,
                             output_path_msg.points.at(i).pose.position.x - output_path_msg.points.at(i - 1).pose.position.x);
        }
        else
        {
            yaw = (std::atan2(output_path_msg.points.at(i + 1).pose.position.y - output_path_msg.points.at(i).pose.position.y,
                              output_path_msg.points.at(i + 1).pose.position.x - output_path_msg.points.at(i).pose.position.x) +
                   std::atan2(output_path_msg.points.at(i).pose.position.y - output_path_msg.points.at(i - 1).pose.position.y,
                              output_path_msg.points.at(i).pose.position.x - output_path_msg.points.at(i - 1).pose.position.x)) /
                  2.0;
        }
        std::cout << "yaw" << yaw<<std::endl;
        tf2_quaternion.setRPY(0, 0, yaw);
        output_path_msg.points.at(i).pose.orientation = tf2::toMsg(tf2_quaternion);
    }

    if (!GoalPathRefiner::getRefinedPath(1.0, 3.14 / 2.0, output_path_msg, input_route_msg.goal_pose, output_path_msg)) {
        ROS_ERROR("Cannot find goal position");
        return false;
    }
    return true;
}

} // namespace behavior_planning

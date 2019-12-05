#include <behavior_velocity_planner/node.hpp>
#include <lanelet2_routing/Route.h>
#include <lanelet2_extension/utility/message_conversion.h>
// #include <lanelet2_extension/utility/query.h>
// #include <lanelet2_extension/visualization/visualization.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
#include <visualization_msgs/MarkerArray.h>
#include <interporation/cubic_spline.hpp>

namespace behavior_planning
{
BehaviorVelocityPlannerNode::BehaviorVelocityPlannerNode() : nh_(), pnh_("~")
{
    path_with_lane_id_sub_ = pnh_.subscribe("input/path_with_lane_id", 1, &BehaviorVelocityPlannerNode::pathWithLaneIdCallback, this);
    perception_sub_ = pnh_.subscribe("input/perception", 1, &BehaviorVelocityPlannerNode::perceptionCallback, this);
    pointcloud_sub_ = pnh_.subscribe("input/pointcloud", 1, &BehaviorVelocityPlannerNode::pointcloudCallback, this);
    map_sub_ = pnh_.subscribe("input/lanelet_map_bin", 10, &BehaviorVelocityPlannerNode::mapCallback, this);
    path_pub_ = pnh_.advertise<autoware_planning_msgs::Path>("output/path", 1);
    debug_viz_pub_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/debug/path", 1);
    pnh_.param("foward_path_length", foward_path_length_, 1000.0);
    pnh_.param("backward_path_length", backward_path_length_, 5.0);
};

void BehaviorVelocityPlannerNode::pathWithLaneIdCallback(const autoware_planning_msgs::PathWithLaneId &input_path_msg)
{
    // if (path_pub_.getNumSubscribers() < 1)
    autoware_planning_msgs::Path output_path_msg;
    if (callback(input_path_msg, *perception_ptr_, *pointcloud_ptr_, output_path_msg))
    {
        output_path_msg.header.frame_id = "map";
        output_path_msg.header.stamp = ros::Time::now();
        path_pub_.publish(output_path_msg);
        publishDebugMarker(output_path_msg, debug_viz_pub_);
    }

    return;
};

void BehaviorVelocityPlannerNode::perceptionCallback(const autoware_perception_msgs::DynamicObjectArray &input_perception_msg)
{
    perception_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(input_perception_msg);

    return;
};

void BehaviorVelocityPlannerNode::pointcloudCallback(const sensor_msgs::PointCloud2 &input_pointcloud_msg)
{
    pointcloud_ptr_ = std::make_shared<sensor_msgs::PointCloud2>(input_pointcloud_msg);

    return;
};

void BehaviorVelocityPlannerNode::mapCallback(const autoware_lanelet2_msgs::MapBin &input_map_msg)
{
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(input_map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

bool BehaviorVelocityPlannerNode::callback(const autoware_planning_msgs::PathWithLaneId &input_path_msg,
                                       const autoware_perception_msgs::DynamicObjectArray &input_perception_msg,
                                       const sensor_msgs::PointCloud2 &input_pointcloud_msg,
                                       autoware_planning_msgs::Path &output_path_msg)
{
    autoware_planning_msgs::Path path;
    for(const auto & path_point: input_path_msg.points){
        path.points.push_back(path_point.point);
    }
    interporatePath(path, foward_path_length_, output_path_msg);
    return true;
}

void BehaviorVelocityPlannerNode::filterPath(const autoware_planning_msgs::Path &path, autoware_planning_msgs::Path &filtered_path)
{
    const double epsilon = 0.01;
    size_t latest_id = 0;
    for (size_t i = 0; i < path.points.size(); ++i)
    {
        double dist;
        if (i != 0)
        {
            const double x = path.points.at(i).pose.position.x - path.points.at(latest_id).pose.position.x;
            const double y = path.points.at(i).pose.position.y - path.points.at(latest_id).pose.position.y;
            dist = std::sqrt(x * x + y * y);
        }
        if (epsilon < dist || i == 0 /*init*/)
        {
            latest_id = i;
            filtered_path.points.push_back(path.points.at(latest_id));
        }
    }
}

void BehaviorVelocityPlannerNode::interporatePath(const autoware_planning_msgs::Path &path, const double length, autoware_planning_msgs::Path &interporated_path)
{
    std::vector<double> x;
    std::vector<double> y;
    std::vector<double> z;
    std::vector<double> v;
    if (50 < path.points.size())
        ROS_WARN("because path size is too large, calculation cost is high. size is %d.", (int)path.points.size());
    for (const auto &path_point : path.points)
    {
        x.push_back(path_point.pose.position.x);
        y.push_back(path_point.pose.position.y);
        z.push_back(path_point.pose.position.z);
        v.push_back(path_point.twist.linear.x);
    }
    std::shared_ptr<Spline4D> spline_ptr;
    spline_ptr = std::make_shared<Spline4D>(x, y, z, v);
    // std::cout<<"st:"<<spline_ptr_->s.back() << std::endl;
    // std::cout<<"point size:"<<path.points.size() << std::endl;
    double s_t;
    for (s_t = 0.0; s_t < std::min(length, spline_ptr->s.back()); s_t += 1.0)
    {
        autoware_planning_msgs::PathPoint path_point;
        std::array<double, 4> state = spline_ptr->calc_trajectory_point(s_t);
        path_point.pose.position.x = state[0];
        path_point.pose.position.y = state[1];
        path_point.pose.position.z = state[2];
        path_point.twist.linear.x = state[3];
        const double yaw = spline_ptr->calc_yaw(s_t);
        tf2::Quaternion tf2_quaternion;
        tf2_quaternion.setRPY(0, 0, yaw);
        path_point.pose.orientation = tf2::toMsg(tf2_quaternion);
        interporated_path.points.push_back(path_point);
    }
    if (spline_ptr->s.back() <= s_t)
        interporated_path.points.push_back(path.points.back());
}

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

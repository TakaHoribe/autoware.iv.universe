#include <behavior_velocity_planner/data_manager.hpp>
#include <lanelet2_extension/utility/message_conversion.h>


namespace behavior_planning
{
void SingletonDataManager::perceptionCallback(const autoware_perception_msgs::DynamicObjectArray &input_perception_msg)
{
}
void SingletonDataManager::pointcloudCallback(const sensor_msgs::PointCloud2 &input_pointcloud_msg)
{
}
void SingletonDataManager::mapCallback(const autoware_lanelet2_msgs::MapBin &input_map_msg)
{
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(input_map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
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
#include <behavior_velocity_planner/data_manager.hpp>
#include <lanelet2_extension/utility/message_conversion.h>


namespace behavior_planning
{
void SingletonDataManager::perceptionCallback(const autoware_perception_msgs::DynamicObjectArray &input_perception_msg)
{
    perception_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(input_perception_msg);
}

void SingletonDataManager::pointcloudCallback(const sensor_msgs::PointCloud2 &input_pointcloud_msg)
{
    pointcloud_ptr_ = std::make_shared<sensor_msgs::PointCloud2>(input_pointcloud_msg);
}

void SingletonDataManager::mapCallback(const autoware_lanelet2_msgs::MapBin &input_map_msg)
{
    lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
    lanelet::utils::conversion::fromBinMsg(input_map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

bool SingletonDataManager::getDynemicObjects(std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects)
{
    if (perception_ptr_ == nullptr)
        return false;
    objects = perception_ptr_;
    return true;
}

bool SingletonDataManager::getNoGroundPointcloud(std::shared_ptr<sensor_msgs::PointCloud2 const> pointcloud)
{
    if (pointcloud_ptr_ == nullptr)
        return false;
    pointcloud = pointcloud_ptr_;
    return true;
}

bool SingletonDataManager::getCurrentSelfPose(geometry_msgs::PoseStamped &pose)
{
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time(0);
    pose.header = header;
    return self_pose_listener_ptr_->getSelfPose(pose.pose, header);
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
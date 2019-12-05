#include "dummy_perception_publisher/node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

DummyPerceptionPublisherNode::DummyPerceptionPublisherNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
    dynamic_object_pub_ = pnh_.advertise<autoware_perception_msgs::DynamicObjectArray>("output/dynamic_object", 1, true);
    pose_pub_ = pnh_.advertise<geometry_msgs::PoseStamped>("output/objects_pose", 1, true);
    pointcloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("output/points_raw", 1, true);
    pose_sub_ = pnh_.subscribe("input/pose", 1, &DummyPerceptionPublisherNode::poseCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &DummyPerceptionPublisherNode::timerCallback, this);
    pnh_.param<double>("velocity", velocity_, double(1.0));
}

void DummyPerceptionPublisherNode::timerCallback(const ros::TimerEvent &)
{
    if (pose_ptr_ == nullptr)
        return;
    autoware_perception_msgs::DynamicObjectArray output_dynamic_object_msg;
    geometry_msgs::PoseStamped output_moved_object_pose;
    sensor_msgs::PointCloud2 output_pointcloud_msg;
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();
    const double move_distance = velocity_ * (ros::Time(header.stamp).toSec() - ros::Time(pose_ptr_->header.stamp).toSec());

    tf2::Transform tf_object_origin2moved_object;
    tf2::Transform tf_map2object_origin;
    tf2::Transform tf_map2moved_object;
    {
        geometry_msgs::Transform ros_object_origin2moved_object;
        ros_object_origin2moved_object.translation.x = move_distance;
        ros_object_origin2moved_object.rotation.x = 0;
        ros_object_origin2moved_object.rotation.y = 0;
        ros_object_origin2moved_object.rotation.z = 0;
        ros_object_origin2moved_object.rotation.w = 1;
        tf2::fromMsg(ros_object_origin2moved_object, tf_object_origin2moved_object);
    }
    tf2::fromMsg(pose_ptr_->pose, tf_map2object_origin);
    tf_map2moved_object = tf_map2object_origin * tf_object_origin2moved_object;
    tf2::toMsg(tf_map2moved_object, output_moved_object_pose.pose);

    const double epsilon = 0.001;
    const double width = 1.0;
    const double length = 2.0;

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (double y = -1.0 * (width / 2.0); y <= ((width / 2.0) + epsilon); y += 0.25)
    {
        for (double x = -1.0 * (length / 2.0); x <= ((length / 2.0) + epsilon); x += 0.25)
        {
            tf2::Transform tf_moved_object2point;
            tf2::Transform tf_base_link2point;
            tf2::Transform tf_base_link2map;

            geometry_msgs::TransformStamped ros_base_link2map;
            ros_base_link2map = tf_buffer_.lookupTransform(/*target*/ "base_link", /*src*/ "map",
                                                             header.stamp, ros::Duration(0.5));
            tf2::fromMsg(ros_base_link2map.transform, tf_base_link2map);

            geometry_msgs::Transform ros_moved_object2point;
            ros_moved_object2point.translation.x = x;
            ros_moved_object2point.translation.y = y;
            ros_moved_object2point.rotation.x = 0;
            ros_moved_object2point.rotation.y = 0;
            ros_moved_object2point.rotation.z = 0;
            ros_moved_object2point.rotation.w = 1;
            tf2::fromMsg(ros_moved_object2point, tf_moved_object2point);
            tf_base_link2point = tf_base_link2map * tf_map2moved_object * tf_moved_object2point;

            pcl::PointXYZ point;
            point.x = tf_base_link2point.getOrigin().x();
            point.y = tf_base_link2point.getOrigin().y();
            point.z = 0.0;
            pointcloud_ptr->push_back(point);
        }
    }
    pcl::toROSMsg(*pointcloud_ptr, output_pointcloud_msg);

    output_moved_object_pose.header = header;
    output_dynamic_object_msg.header = header;
    output_pointcloud_msg.header.frame_id = "base_link";
    output_pointcloud_msg.header.stamp = header.stamp;
    pointcloud_pub_.publish(output_pointcloud_msg);
    dynamic_object_pub_.publish(output_dynamic_object_msg);
    pose_pub_.publish(output_moved_object_pose);
}

void DummyPerceptionPublisherNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_msg)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = input_msg->header;

    if (input_msg->header.frame_id != "map")
    {
        tf2::Transform tf_map2input_world;
        tf2::Transform tf_input_world2object;
        tf2::Transform tf_map2objet;
        try
        {
            geometry_msgs::TransformStamped ros_map2input_world;
            ros_map2input_world = tf_buffer_.lookupTransform(/*target*/ "map", /*src*/ input_msg->header.frame_id,
                                                             input_msg->header.stamp, ros::Duration(0.5));
            tf2::fromMsg(ros_map2input_world.transform, tf_map2input_world);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
        tf2::fromMsg(input_msg->pose.pose, tf_input_world2object);
        tf_map2objet = tf_map2input_world * tf_input_world2object;
        tf2::toMsg(tf_map2objet, pose_msg.pose);
    }
    pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(pose_msg);
}
#include "dummy_perception_publisher/node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

DummyPerceptionPublisherNode::DummyPerceptionPublisherNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
    dynamic_object_pub_ = pnh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>("output/dynamic_object", 1, true);
    pointcloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("output/points_raw", 1, true);
    pedestrian_pose_sub_ = pnh_.subscribe("input/pedestrian/pose", 1, &DummyPerceptionPublisherNode::pedestrianPoseCallback, this);
    car_pose_sub_ = pnh_.subscribe("input/car/pose", 1, &DummyPerceptionPublisherNode::carPoseCallback, this);
    timer_ = nh_.createTimer(ros::Duration(0.1), &DummyPerceptionPublisherNode::timerCallback, this);
    pnh_.param<double>("velocity", velocity_, double(1.0));
    pnh_.param<double>("visible_range", visible_range_, double(100.0));
}

void DummyPerceptionPublisherNode::timerCallback(const ros::TimerEvent &)
{
    autoware_perception_msgs::DynamicObjectWithFeatureArray output_dynamic_object_msg;
    geometry_msgs::PoseStamped output_moved_object_pose;
    sensor_msgs::PointCloud2 output_pointcloud_msg;
    std_msgs::Header header;
    header.frame_id = "map";
    header.stamp = ros::Time::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);

    // pedestrian
    {
        std::vector<size_t> delete_idxs;
        for (size_t i = 0; i < pedestrian_pose_ptrs_.size(); ++i)
        {
            if (pedestrian_pose_ptrs_.at(i) != nullptr)
            {
                const double move_distance = velocity_ * (ros::Time(header.stamp).toSec() - ros::Time(pedestrian_pose_ptrs_.at(i)->header.stamp).toSec());

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
                tf2::fromMsg(pedestrian_pose_ptrs_.at(i)->pose, tf_map2object_origin);
                tf_map2moved_object = tf_map2object_origin * tf_object_origin2moved_object;
                tf2::toMsg(tf_map2moved_object, output_moved_object_pose.pose);

                const double epsilon = 0.001;
                const double width = 1.0;
                const double length = 1.0;

                // pointcloud
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
                        ros_moved_object2point.translation.z = output_moved_object_pose.pose.position.z;
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

                // dynamic object
                autoware_perception_msgs::DynamicObjectWithFeature feature_object;
                feature_object.object.semantic.type = autoware_perception_msgs::Semantic::PEDESTRIAN;
                feature_object.object.semantic.confidence = 1.0;
                feature_object.object.state.pose_covariance.pose = output_moved_object_pose.pose;
                feature_object.object.state.orientation_reliable = false;
                feature_object.object.state.twist_covariance.twist.linear.x = velocity_;
                feature_object.object.state.twist_reliable = true;
                feature_object.object.state.acceleration_reliable = true;
                feature_object.object.shape.type = autoware_perception_msgs::Shape::CYLINDER;
                feature_object.object.shape.dimensions.x = std::max(width, length);
                feature_object.object.shape.dimensions.y = std::max(width, length);
                feature_object.object.shape.dimensions.z = 2.0;
                output_dynamic_object_msg.feature_objects.push_back(feature_object);

                // check delete idx
                tf2::Transform tf_base_link2moved_object;
                tf2::Transform tf_base_link2map;
                geometry_msgs::TransformStamped ros_base_link2map;
                ros_base_link2map = tf_buffer_.lookupTransform(/*target*/ "base_link", /*src*/ "map",
                                                               header.stamp, ros::Duration(0.5));
                tf2::fromMsg(ros_base_link2map.transform, tf_base_link2map);
                tf_base_link2moved_object = tf_base_link2map * tf_map2moved_object;
                double dist = std::sqrt(tf_base_link2moved_object.getOrigin().x() * tf_base_link2moved_object.getOrigin().x() +
                                        tf_base_link2moved_object.getOrigin().y() * tf_base_link2moved_object.getOrigin().y());
                if (visible_range_ < dist)
                    delete_idxs.push_back(i);
            }
        }
        // delete
        for (int delete_idx = delete_idxs.size() - 1; 0 <= delete_idx; --delete_idx)
        {
            pedestrian_pose_ptrs_.erase(pedestrian_pose_ptrs_.begin() + delete_idxs.at(delete_idx));
        }
    }

    // car
    {
        std::vector<size_t> delete_idxs;
        for (size_t i = 0; i < car_pose_ptrs_.size(); ++i)
        {
            if (car_pose_ptrs_.at(i) != nullptr)
            {
                const double move_distance = velocity_ * (ros::Time(header.stamp).toSec() - ros::Time(car_pose_ptrs_.at(i)->header.stamp).toSec());

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
                tf2::fromMsg(car_pose_ptrs_.at(i)->pose, tf_map2object_origin);
                tf_map2moved_object = tf_map2object_origin * tf_object_origin2moved_object;
                tf2::toMsg(tf_map2moved_object, output_moved_object_pose.pose);

                const double epsilon = 0.001;
                const double width = 2.5;
                const double length = 4.0;

                // pointcloud
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
                        ros_moved_object2point.translation.z = output_moved_object_pose.pose.position.z;
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

                // dynamic object
                autoware_perception_msgs::DynamicObjectWithFeature feature_object;
                feature_object.object.semantic.type = autoware_perception_msgs::Semantic::CAR;
                feature_object.object.semantic.confidence = 1.0;
                feature_object.object.state.pose_covariance.pose = output_moved_object_pose.pose;
                feature_object.object.state.orientation_reliable = false;
                feature_object.object.state.twist_covariance.twist.linear.x = velocity_;
                feature_object.object.state.twist_reliable = true;
                feature_object.object.state.acceleration_reliable = true;
                feature_object.object.shape.type = autoware_perception_msgs::Shape::BOUNDING_BOX;
                feature_object.object.shape.dimensions.x = length;
                feature_object.object.shape.dimensions.y = width;
                feature_object.object.shape.dimensions.z = 2.0;
                output_dynamic_object_msg.feature_objects.push_back(feature_object);

                // check delete idx
                tf2::Transform tf_base_link2moved_object;
                tf2::Transform tf_base_link2map;
                geometry_msgs::TransformStamped ros_base_link2map;
                ros_base_link2map = tf_buffer_.lookupTransform(/*target*/ "base_link", /*src*/ "map",
                                                               header.stamp, ros::Duration(0.5));
                tf2::fromMsg(ros_base_link2map.transform, tf_base_link2map);
                tf_base_link2moved_object = tf_base_link2map * tf_map2moved_object;
                double dist = std::sqrt(tf_base_link2moved_object.getOrigin().x() * tf_base_link2moved_object.getOrigin().x() +
                                        tf_base_link2moved_object.getOrigin().y() * tf_base_link2moved_object.getOrigin().y());
                if (visible_range_ < dist)
                    delete_idxs.push_back(i);
                        }
        }
        // delete
        for (int delete_idx = delete_idxs.size() - 1; 0 <= delete_idx; --delete_idx)
            car_pose_ptrs_.erase(car_pose_ptrs_.begin() + delete_idxs.at(delete_idx));
    }
    pcl::toROSMsg(*pointcloud_ptr, output_pointcloud_msg);

    // create output header
    output_moved_object_pose.header = header;
    output_dynamic_object_msg.header = header;
    output_pointcloud_msg.header.frame_id = "base_link";
    output_pointcloud_msg.header.stamp = header.stamp;

    // publish
    pointcloud_pub_.publish(output_pointcloud_msg);
    dynamic_object_pub_.publish(output_dynamic_object_msg);
}

void DummyPerceptionPublisherNode::pedestrianPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_msg)
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
    pedestrian_pose_ptrs_.push_back(std::make_shared<geometry_msgs::PoseStamped>(pose_msg));
}

void DummyPerceptionPublisherNode::carPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &input_msg)
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
    car_pose_ptrs_.push_back(std::make_shared<geometry_msgs::PoseStamped>(pose_msg));
}
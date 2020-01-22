#include "dummy_perception_publisher/node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

DummyPerceptionPublisherNode::DummyPerceptionPublisherNode() : nh_(),
                                                               pnh_("~"),
                                                               tf_listener_(tf_buffer_),
                                                               pedestrian_sync_(SyncPolicy(10), pedestrian_pose_sub_, pedestrian_twist_sub_),
                                                               car_sync_(SyncPolicy(10), car_pose_sub_, car_twist_sub_),
                                                               object_id(0)
{
    dynamic_object_pub_ = pnh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>("output/dynamic_object", 1, true);
    pointcloud_pub_ = pnh_.advertise<sensor_msgs::PointCloud2>("output/points_raw", 1, true);

    pedestrian_pose_sub_.subscribe(pnh_, "input/pedestrian/pose", 1);
    pedestrian_twist_sub_.subscribe(pnh_, "input/pedestrian/twist", 1);
    pedestrian_sync_.registerCallback(boost::bind(&DummyPerceptionPublisherNode::pedestrianPoseCallback, this, _1, _2));

    car_pose_sub_.subscribe(pnh_, "input/car/pose", 1);
    car_twist_sub_.subscribe(pnh_, "input/car/twist", 1);
    car_sync_.registerCallback(boost::bind(&DummyPerceptionPublisherNode::carPoseCallback, this, _1, _2));

    object_id_sub_ = pnh_.subscribe("input/object_id", 1, &DummyPerceptionPublisherNode::callbackObjectId, this);
    object_reset_id_sub_ = pnh_.subscribe("input/object_reset_id", 1, &DummyPerceptionPublisherNode::callbackObjectResetId, this);

    timer_ = nh_.createTimer(ros::Duration(0.1), &DummyPerceptionPublisherNode::timerCallback, this);
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
        for (size_t i = 0; i < pedestrian_poses_.size(); ++i)
        {
            const double move_distance = std::get<TWIST>(pedestrian_poses_.at(i)).twist.linear.x * (ros::Time(header.stamp).toSec() - ros::Time(std::get<POSE>(pedestrian_poses_.at(i)).header.stamp).toSec());

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
            tf2::fromMsg(std::get<POSE>(pedestrian_poses_.at(i)).pose, tf_map2object_origin);
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
            feature_object.object.state.twist_covariance.twist.linear.x = std::get<TWIST>(pedestrian_poses_.at(i)).twist.linear.x;
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
        // delete
        for (int delete_idx = delete_idxs.size() - 1; 0 <= delete_idx; --delete_idx)
        {
            pedestrian_poses_.erase(pedestrian_poses_.begin() + delete_idxs.at(delete_idx));
        }
    }

    // car
    {
        std::vector<size_t> delete_idxs;
        for (size_t i = 0; i < car_poses_.size(); ++i)
        {
            const double move_distance = std::get<TWIST>(car_poses_.at(i)).twist.linear.x * (ros::Time(header.stamp).toSec() - ros::Time(std::get<POSE>(car_poses_.at(i)).header.stamp).toSec());

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
            tf2::fromMsg(std::get<POSE>(car_poses_.at(i)).pose, tf_map2object_origin);
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
            feature_object.object.state.twist_covariance.twist.linear.x = std::get<TWIST>(car_poses_.at(i)).twist.linear.x;
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
        // delete
        for (int delete_idx = delete_idxs.size() - 1; 0 <= delete_idx; --delete_idx)
            car_poses_.erase(car_poses_.begin() + delete_idxs.at(delete_idx));
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

void DummyPerceptionPublisherNode::pedestrianPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &input_pose_msg,
                                                          const geometry_msgs::TwistStamped::ConstPtr &input_twist_msg)
{
    std::tuple<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, int> tuple;
    convert2Tuple(input_pose_msg, input_twist_msg, object_id, tuple);
    pedestrian_poses_.push_back(tuple);
}

void DummyPerceptionPublisherNode::carPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &input_pose_msg,
                                                   const geometry_msgs::TwistStamped::ConstPtr &input_twist_msg)
{
    std::tuple<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, int> tuple;
    convert2Tuple(input_pose_msg, input_twist_msg, object_id, tuple);
    car_poses_.push_back(tuple);
}

void DummyPerceptionPublisherNode::convert2Tuple(const geometry_msgs::PoseStamped::ConstPtr &input_pose_msg,
                                                 const geometry_msgs::TwistStamped::ConstPtr &input_twist_msg,
                                                 int object_id,
                                                 std::tuple<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped, int> &output)
{
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = input_pose_msg->header;

    if (input_pose_msg->header.frame_id != "map")
    {
        tf2::Transform tf_map2input_world;
        tf2::Transform tf_input_world2object;
        tf2::Transform tf_map2objet;
        try
        {
            geometry_msgs::TransformStamped ros_map2input_world;
            ros_map2input_world = tf_buffer_.lookupTransform(/*target*/ "map", /*src*/ input_pose_msg->header.frame_id,
                                                             input_pose_msg->header.stamp, ros::Duration(0.5));
            tf2::fromMsg(ros_map2input_world.transform, tf_map2input_world);
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return;
        }
        tf2::fromMsg(input_pose_msg->pose, tf_input_world2object);
        tf_map2objet = tf_map2input_world * tf_input_world2object;
        tf2::toMsg(tf_map2objet, pose_msg.pose);
    }
    output = std::make_tuple(pose_msg, *input_twist_msg, object_id);
}

void DummyPerceptionPublisherNode::callbackObjectId(const std_msgs::Int32ConstPtr &msg)
{
    int _object_id = msg->data;
    if(_object_id < 0){
        ROS_WARN("invalid object id. Ignore input.");
    }else
    {
        object_id = _object_id;
    }
}

void DummyPerceptionPublisherNode::callbackObjectResetId(const std_msgs::Int32ConstPtr &msg)
{
    int _object_id = msg->data;
    if(_object_id < -1){
        ROS_WARN("invalid object reset id. Ignore input.");
    }else if(_object_id==-1)
    {
        ResetAllObject();
    }else
    {
        ResetObject(_object_id);
    }
}

void DummyPerceptionPublisherNode::ResetAllObject(void)
{
    car_poses_.clear();
    pedestrian_poses_.clear();
}

void DummyPerceptionPublisherNode::ResetObject(int obj_id)
{
        for (int i = car_poses_.size()-1; i >= 0; --i)
    {
        if (obj_id == std::get<INT>(car_poses_.at(i)))
        {
            car_poses_.erase(car_poses_.begin() + i);//reset car object id i
        }
    }

        for (int i = pedestrian_poses_.size()-1; i >= 0; --i)
    {
        if (obj_id == std::get<INT>(pedestrian_poses_.at(i)))
        {
            pedestrian_poses_.erase(pedestrian_poses_.begin() + i);//reset car object id i
        }
    }
}
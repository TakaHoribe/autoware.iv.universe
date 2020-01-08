#pragma once
#include <ros/ros.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tuple>
#include <memory>

class DummyPerceptionPublisherNode
{
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher dynamic_object_pub_;

  message_filters::Subscriber<geometry_msgs::PoseStamped> pedestrian_pose_sub_;
  message_filters::Subscriber<geometry_msgs::TwistStamped> pedestrian_twist_sub_;
  message_filters::Subscriber<geometry_msgs::PoseStamped> car_pose_sub_;
  message_filters::Subscriber<geometry_msgs::TwistStamped> car_twist_sub_;
  typedef message_filters::sync_policies::ExactTime<geometry_msgs::PoseStamped,
                                                    geometry_msgs::TwistStamped>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync pedestrian_sync_;
  Sync car_sync_;

  ros::Timer timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  enum
  {
    POSE = 0,
    TWIST = 1
  };
  std::vector<std::tuple<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped>> pedestrian_poses_;
  std::vector<std::tuple<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped>> car_poses_;
  double visible_range_;

  void timerCallback(const ros::TimerEvent &);
  void pedestrianPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, const geometry_msgs::TwistStamped::ConstPtr &twist_msg);
  void carPoseCallback(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, const geometry_msgs::TwistStamped::ConstPtr &twist_msg);
  void convert2Tuple(const geometry_msgs::PoseStamped::ConstPtr &pose_msg, const geometry_msgs::TwistStamped::ConstPtr &twist_msg,
                       std::tuple<geometry_msgs::PoseStamped, geometry_msgs::TwistStamped> &output);

public:
  DummyPerceptionPublisherNode();
  ~DummyPerceptionPublisherNode(){};
};
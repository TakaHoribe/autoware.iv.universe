#pragma once
#include <ros/ros.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <memory>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class DummyPerceptionPublisherNode
{
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pointcloud_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher dynamic_object_pub_;
  ros::Subscriber pose_sub_;
  ros::Timer timer_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::shared_ptr<geometry_msgs::PoseStamped> pose_ptr_;
  double velocity_;
  void timerCallback(const ros::TimerEvent &);
  void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & pose_msg);

public:
  DummyPerceptionPublisherNode();
  ~DummyPerceptionPublisherNode(){};
};
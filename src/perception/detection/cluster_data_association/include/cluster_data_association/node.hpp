#pragma once
#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "autoware_perception_msgs/DynamicObjectWithFeatureArray.h"
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

namespace cluster_data_association {
class ClusterDataAssociationNode {
 public:
  ClusterDataAssociationNode();
  ~ClusterDataAssociationNode(){};

 private:
  void clusterCallback(const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& input_cluster0_msg,
                       const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& input_cluster1_msg);
  void getCentroid(const pcl::PointCloud<pcl::PointXYZ>& pointcloud, pcl::PointXYZ& centroid);

  ros::NodeHandle nh_, pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Publisher associated_cluster_pub_;
  message_filters::Subscriber<autoware_perception_msgs::DynamicObjectWithFeatureArray> cluster0_sub_;
  message_filters::Subscriber<autoware_perception_msgs::DynamicObjectWithFeatureArray> cluster1_sub_;
  typedef message_filters::sync_policies::ApproximateTime<autoware_perception_msgs::DynamicObjectWithFeatureArray,
                                                          autoware_perception_msgs::DynamicObjectWithFeatureArray>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
  // ROS Parameters
};

}  // namespace cluster_data_association


#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <chrono>
#include <cluster_data_association/node.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace cluster_data_association {

ClusterDataAssociationNode::ClusterDataAssociationNode()
    : nh_(""),
      pnh_("~"),
      tf_listener_(tf_buffer_),
      cluster0_sub_(pnh_, "input/clusters0", 1),
      cluster1_sub_(pnh_, "input/clusters1", 1),
      sync_(SyncPolicy(10), cluster0_sub_, cluster1_sub_)
{
  sync_.registerCallback(boost::bind(&ClusterDataAssociationNode::clusterCallback, this, _1, _2));

  associated_cluster_pub_ =
      nh_.advertise<autoware_perception_msgs::DynamicObjectWithFeatureArray>("associated_clusters", 10);
}

void ClusterDataAssociationNode::clusterCallback(
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& input_cluster0_msg,
    const autoware_perception_msgs::DynamicObjectWithFeatureArray::ConstPtr& input_cluster1_msg)
{
  // Guard
  if (associated_cluster_pub_.getNumSubscribers() < 1) return;

  // build output msg
  autoware_perception_msgs::DynamicObjectWithFeatureArray output_msg;

  for (size_t i = 0; i < input_cluster0_msg->feature_objects.size(); ++i) {
    output_msg.feature_objects.push_back(input_cluster0_msg->feature_objects.at(i));
  }
  for (size_t i = 0; i < input_cluster1_msg->feature_objects.size(); ++i) {
    output_msg.feature_objects.push_back(input_cluster1_msg->feature_objects.at(i));
  }
  // publish output msg
  associated_cluster_pub_.publish(output_msg);
}

void ClusterDataAssociationNode::getCentroid(const pcl::PointCloud<pcl::PointXYZ>& pointcloud,
                                             pcl::PointXYZ& centroid) {
  centroid.x = 0;
  centroid.y = 0;
  centroid.z = 0;
  for (const auto& point : pointcloud) {
    centroid.x += point.x;
    centroid.y += point.y;
    centroid.z += point.z;
  }
  centroid.x = centroid.x / (double)pointcloud.size();
  centroid.y = centroid.y / (double)pointcloud.size();
  centroid.z = centroid.z / (double)pointcloud.size();
  return;
}
}  // namespace cluster_data_association

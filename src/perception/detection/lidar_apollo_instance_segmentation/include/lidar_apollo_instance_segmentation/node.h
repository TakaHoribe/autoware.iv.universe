#pragma once
#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObjectWithFeatureArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <memory>

class LidarInstanceSegmentationInterface {
 public:
  LidarInstanceSegmentationInterface(){}
  virtual ~LidarInstanceSegmentationInterface(){}
  virtual bool detectDynamicObjects(const sensor_msgs::PointCloud2& input,
                                    autoware_perception_msgs::DynamicObjectWithFeatureArray& output) = 0;
};

class LidarInstanceSegmentationNode {
 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber pointcloud_sub_;
  ros::Publisher dynamic_objects_pub_;
  void pointCloudCallback(const sensor_msgs::PointCloud2& msg);
  std::shared_ptr<LidarInstanceSegmentationInterface> detector_ptr_;

 public:
  LidarInstanceSegmentationNode();
  ~LidarInstanceSegmentationNode() {}
};

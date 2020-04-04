#pragma once
#include <ros/ros.h>
#include "autoware_perception_msgs/DynamicObjectWithFeatureArray.h"

class Debugger {
 public:
  Debugger();
  ~Debugger(){};
  ros::Publisher instance_pointcloud_pub_;
  void publishColoredPointCloud(const autoware_perception_msgs::DynamicObjectWithFeatureArray& input);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
};
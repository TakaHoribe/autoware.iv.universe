#pragma once
#include <autoware_planning_msgs/Path.h>
#include <ros/ros.h>
#include <memory>
#include <tf2_ros/transform_listener.h>

namespace behavior_planner
{
class BehaviorPlanningSchedulerNode
{
protected:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher path_pub_;
  ros::Subscriber path_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  void pathCallback(const autoware_planning_msgs::Path &input_path_msg);

public:
  BehaviorPlanningSchedulerNode();
  virtual ~BehaviorPlanningSchedulerNode(){};
};
} // namespace motion_planner
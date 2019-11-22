#pragma once
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <ros/ros.h>
#include <memory>
#include <tf2_ros/transform_listener.h>

namespace motion_planner
{
class BasePlannerNode
{
protected:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher path_pub_;
  ros::Subscriber path_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  virtual bool callback(const autoware_planning_msgs::Path &input_path_msg, autoware_planning_msgs::Path &output_path_msg) = 0;
  void pathCallback(const autoware_planning_msgs::Path &input_path_msg);
  bool getSelfPose(geometry_msgs::TransformStamped& self_pose, const std_msgs::Header &header);
  bool getCurrentSelfPose(geometry_msgs::TransformStamped& self_pose);

public:
  BasePlannerNode();
  virtual ~BasePlannerNode(){};
};
} // namespace motion_planner
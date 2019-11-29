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
  ros::Publisher trajectory_pub_;
  ros::Subscriber path_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Timer timer_;
  std::shared_ptr<autoware_planning_msgs::Path> path_ptr_;
  virtual void callback(const autoware_planning_msgs::Path &input_path_msg, autoware_planning_msgs::Trajectory &output_trajectory_msg) = 0;
  void timerCallback(const ros::TimerEvent &e);
  void pathCallback(const autoware_planning_msgs::Path &input_path_msg);
  bool getSelfPose(geometry_msgs::Pose& self_pose, const std_msgs::Header &header);
  bool getCurrentSelfPose(geometry_msgs::Pose& self_pose);

public:
  BasePlannerNode();
  virtual ~BasePlannerNode(){};
};
} // namespace motion_planner
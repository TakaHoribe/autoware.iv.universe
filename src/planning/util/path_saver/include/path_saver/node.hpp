#pragma once
#include <ros/ros.h>
#include <autoware_planning_msgs/Path.h>
#include <tf2_ros/transform_listener.h>
#include <fstream>
#include <memory>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
class PathSaverNode
{
private:
  ros::NodeHandle nh_, pnh_;
  ros::Subscriber twist_sub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  ros::Timer timer_;
  void timerCallback(const ros::TimerEvent &);
  void twistCallback(const geometry_msgs::TwistStamped::ConstPtr &input_twist_msg);
  geometry_msgs::TwistStamped::ConstPtr twist_;
  geometry_msgs::Twist last_saved_twist_;
  geometry_msgs::Pose last_saved_pose_;
  std::shared_ptr<ros::Time> last_saved_time_;
  std::ofstream ofs_;
  double dist_threshold_;
  double time_threshold_;

public:
  PathSaverNode();
  ~PathSaverNode(){};
};
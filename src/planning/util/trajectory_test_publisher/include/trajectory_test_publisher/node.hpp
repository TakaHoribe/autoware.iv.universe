#pragma once
#include <ros/ros.h>
#include <autoware_planning_msgs/Trajectory.h>

class TrajectoriTestPublisherNode
{
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_;
  ros::Timer timer_;
  void timerCallback(const ros::TimerEvent &);

public:
  TrajectoriTestPublisherNode();
  ~TrajectoriTestPublisherNode(){};
};
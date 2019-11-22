#pragma once
#include <ros/ros.h>
#include <autoware_planning_msgs/Path.h>

class PathTestPublisherNode
{
private:
  ros::NodeHandle nh_, pnh_;
  ros::Publisher pub_;
  ros::Timer timer_;
  void timerCallback(const ros::TimerEvent &);

public:
  PathTestPublisherNode();
  ~PathTestPublisherNode(){};
};
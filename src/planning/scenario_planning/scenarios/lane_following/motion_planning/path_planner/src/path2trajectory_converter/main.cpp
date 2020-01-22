#include <ros/ros.h>
#include "path2trajectory_converter/node.hpp"


int main(int argc, char **argv)
{ 
  ros::init(argc, argv, "path2trajectory_converter");
  path_planner::Path2Trajectory node;

  ros::spin();

  return 0;
};

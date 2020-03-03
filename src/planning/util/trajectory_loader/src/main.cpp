#include <ros/ros.h>
#include "trajectory_loader/node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "trajectory_loader");
  TrajectoryLoaderNode node;

  ros::spin();

  return 0;
};
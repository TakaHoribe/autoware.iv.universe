#include <ros/ros.h>
#include "path_loader/node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_loader");
  PathLoaderNode node;

  ros::spin();

  return 0;
};
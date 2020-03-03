#include <ros/ros.h>
#include "path_saver/node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "path_saver");
  PathSaverNode node;

  ros::spin();

  return 0;
};
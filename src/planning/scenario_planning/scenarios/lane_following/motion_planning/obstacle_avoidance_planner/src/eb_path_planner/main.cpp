#include <ros/ros.h>
#include "eb_path_planner/node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "eb_path_planner");
  path_planner::EBPathPlannerNode node;

  ros::spin();

  return 0;
};

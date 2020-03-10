#include <ros/ros.h>
#include "eb_path_planner/node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "eb_path_planner");
  EBPathPlannerNode node;

  ros::spin();

  return 0;
};

#include <ros/ros.h>
#include "behavior_planning_scheduler/node.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "behavior_planning_scheduler");
  behavior_planner::BehaviorPlanningSchedulerNode node;

  ros::spin();

  return 0;
};
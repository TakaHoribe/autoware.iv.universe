#include <ros/ros.h>

#include "scenario_selector_node.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "scenario_selector");

  ScenarioSelectorNode node;

  ros::spin();

  return 0;
}

#include <ros/ros.h>

#include <autoware_state_monitor/autoware_state_monitor_node.h>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "autoware_state_monitor");

  AutowareStateMonitorNode node;

  ros::spin();

  return 0;
}

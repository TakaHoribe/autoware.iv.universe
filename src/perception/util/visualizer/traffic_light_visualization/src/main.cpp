#include <ros/ros.h>
#include <traffic_light_roi_visualizer/node.hpp>

int main(int argc, char** argv) {
  ros::init(argc, argv, "traffic_light_roi_visualizer_node");
  traffic_light::TrafficLightRoiVisualizer node;

  ros::spin();
  return 0;
}

#include <ros/ros.h>
#include "traffic_light_classifier/node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "traffic_light_classifier_node");
  traffic_light::TrafficLightClassifierNode node;
  ros::spin();
  return 0;
}

#include <ros/ros.h>
#include "node.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "traffic_light_roi_image_saver_node");
  traffic_light::TrafficLightRoiImageSaver node;

  ros::spin();
  return 0;
}

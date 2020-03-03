#include "traffic_light_fine_detector/traffic_light_fine_detector.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "traffic_light_fine_detector");

  traffic_light::TrafficLightFineDetectorNode node;
  node.run();

  return 0;
}

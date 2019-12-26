#include "new_trafficlight_recognizer/tlr_detector.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "trafficlight_detector");

  new_trafficlight_recognizer::TrafficLightDetectorNode node;
  node.run();

  return 0;
}

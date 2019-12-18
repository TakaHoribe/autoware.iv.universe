#include "new_trafficlight_recognizer/traffic_light_classifier.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trafficlight_classifier");

    new_trafficlight_recognizer::TrafficLightClassifierNode classifier;
    classifier.run();

    return 0;
}

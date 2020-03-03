#pragma once

#include <autoware_traffic_light_msgs/LampState.h>
#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace traffic_light {
class ClassifierInterface {
 public:
  virtual bool getLampState(const cv::Mat& input_image,
                            std::vector<autoware_traffic_light_msgs::LampState>& states) = 0;
};
}  // namespace traffic_light
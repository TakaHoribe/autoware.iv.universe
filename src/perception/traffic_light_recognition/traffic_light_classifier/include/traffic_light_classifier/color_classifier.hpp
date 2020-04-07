#pragma once

#include <autoware_traffic_light_msgs/LampState.h>
#include <cv_bridge/cv_bridge.h>
#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <traffic_light_classifier/HSVFilterConfig.h>
#include <traffic_light_classifier/classifier_interface.hpp>

#include <opencv/cv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace traffic_light {
class ColorClassifier : public ClassifierInterface {
 public:
  ColorClassifier();
  ~ColorClassifier(){};

  bool getLampState(const cv::Mat& input_image, std::vector<autoware_traffic_light_msgs::LampState>& states) override;

 private:
  bool filterHSV(const cv::Mat& input_image, cv::Mat& green_image, cv::Mat& yellow_image, cv::Mat& red_image);
  void parametersCallback(traffic_light_classifier::HSVFilterConfig& config, uint32_t level);

 private:
  enum HSV {
    Hue = 0,
    Sat = 1,
    Val = 2,
  };
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::Publisher image_pub_;
  dynamic_reconfigure::Server<traffic_light_classifier::HSVFilterConfig> dynamic_reconfigure_;
  double ratio_threshold_;
  cv::Scalar min_hsv_green_;
  cv::Scalar max_hsv_green_;
  cv::Scalar min_hsv_yellow_;
  cv::Scalar max_hsv_yellow_;
  cv::Scalar min_hsv_red_;
  cv::Scalar max_hsv_red_;
};

}  // namespace traffic_light
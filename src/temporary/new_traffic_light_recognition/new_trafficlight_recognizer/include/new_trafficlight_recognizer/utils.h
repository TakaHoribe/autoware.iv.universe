#ifndef TRAFFICLIGHT_RECOGNIZER_UTILS_H
#define TRAFFICLIGHT_RECOGNIZER_UTILS_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/RegionOfInterest.h>
#include <autoware_traffic_light_msgs/TrafficLightStateArray.h>
#include <autoware_traffic_light_msgs/TrafficLightState.h>
#include <autoware_traffic_light_msgs/LampStateArray.h>
#include <autoware_traffic_light_msgs/LampState.h>
#include <autoware_traffic_light_msgs/StampedRoiArray.h>
#include <autoware_traffic_light_msgs/StampedRoi.h>

#include <eigen_conversions/eigen_msg.h>

namespace new_trafficlight_recognizer
{

  class Utils
  {
  public:

    enum Position {
      LEFTTOP,
      LEFTBOTTOM,
      RIGHTTOP,
      RIGHTBOTTOM,
    };

    int get_area(const cv::Mat& image);

    bool merge_msg(const autoware_traffic_light_msgs::LampStateArray& color_lamp_states,
                   const autoware_traffic_light_msgs::LampStateArray& arrow_lamp_states,
                   autoware_traffic_light_msgs::LampStateArray& lamp_states);

    bool rosmsg2cvmat(const sensor_msgs::Image::ConstPtr& image_msg, cv::Mat& image);

    bool roismsg2cvrects(const std::vector<sensor_msgs::RegionOfInterest>& rois,
                         std::vector<cv::Rect>& rects);

    bool cvrects2roismsg(const std::vector<cv::Rect>& rects,
                         std::vector<sensor_msgs::RegionOfInterest>& rois);

    bool fit_in_frame(cv::Point& lt, cv::Point& rb, const cv::Size& size);


    inline float sigmoid(float x, float a=200, float movement=0.02)
    {
      return(1 - (1.0 / (1.0 + exp(a * (-x + movement)))));
    }

    template<typename T>
      inline bool in_vector(const T& c, const typename T::value_type& v)
      {
        return(c.end() != std::find(c.begin(), c.end(), v));
      }

  };

}

#endif

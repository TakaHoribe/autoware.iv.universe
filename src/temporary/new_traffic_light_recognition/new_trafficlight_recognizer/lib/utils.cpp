#include "new_trafficlight_recognizer/utils.h"

namespace new_trafficlight_recognizer
{

  int Utils::get_area(const cv::Mat& image)
  {
    int cnt = 0;
    for (int y=0; y<image.rows; ++y) {
      for (int x=0; x<image.cols; ++x) {
        if (image.at<uchar>(y, x) > 127)
          cnt++;
      }
    }
    return cnt;
  }


  bool Utils::merge_msg(const autoware_traffic_light_msgs::LampStateArray& color_lamp_states,
                        const autoware_traffic_light_msgs::LampStateArray& arrow_lamp_states,
                        autoware_traffic_light_msgs::LampStateArray& lamp_states)
  {
    try
      {
        for (auto state : color_lamp_states.states)
          {
            lamp_states.states.push_back(state);
          }

        for (auto state : arrow_lamp_states.states)
          {
            lamp_states.states.push_back(state);
          }
      }
    catch (...)
      {
        return false;
      }

    return true;
  }

  bool Utils::rosmsg2cvmat(const sensor_msgs::Image::ConstPtr& image_msg, cv::Mat& image)
  {
    try
      {
        cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(image_msg, "bgr8");
        image = cv_image->image;
      }
    catch (cv_bridge::Exception& e)
      {
        ROS_ERROR("Failed to convert sensor_msgs::Image to cv::Mat \n%s", e.what());
        return false;
      }

    return true;
  }


  bool Utils::roismsg2cvrects(const std::vector<sensor_msgs::RegionOfInterest>& rois,
                              std::vector<cv::Rect>& rects)
  {
    for (auto roi : rois)
      {
        rects.push_back
          (cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));
      }

    return true;
  }


  bool Utils::cvrects2roismsg(const std::vector<cv::Rect>& rects,
                              std::vector<sensor_msgs::RegionOfInterest>& rois)

  {
    for (auto rect : rects)
      {
        sensor_msgs::RegionOfInterest roi;
        roi.x_offset = rect.x;
        roi.y_offset = rect.y;
        roi.width = rect.width;
        roi.height = rect.height;
        rois.push_back(roi);
      }

    return true;
  }


  bool Utils::fit_in_frame(cv::Point& lt, cv::Point& rb, const cv::Size& size)
  {
    try
      {
        if (rb.x > size.width) rb.x = size.width;
        if (rb.y > size.height) rb.y = size.height;
        if (lt.x < 0)  lt.x = 0;
        if (lt.y < 0) lt.y = 0;
      }
    catch (cv::Exception& e )
      {
        ROS_ERROR("Failed to fit bounding rect in size [%d, %d] \n%s", size.width, size.height, e.what());
        return false;
      }

    return true;
  }

} // new_trafficlight_recognizer

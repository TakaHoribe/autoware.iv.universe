#include "traffic_light_classifier/node.hpp"
#include <iostream>

namespace traffic_light
{
TrafficLightClassifierNode::TrafficLightClassifierNode() : nh_(""),
                                                           pnh_("~"),
                                                           image_transport_(pnh_),
                                                           image_sub_(image_transport_, "input/image", 1),
                                                           roi_sub_(pnh_, "input/rois", 1),
                                                           sync_(SyncPolicy(10), image_sub_, roi_sub_)
{
  sync_.registerCallback(boost::bind(&TrafficLightClassifierNode::imageRoiCallback, this, _1, _2));
  tl_states_pub_ = pnh_.advertise<autoware_traffic_light_msgs::TrafficLightStateArray>("output/traffic_light_states", 1);
  classifier_ptr_ = std::make_shared<ColorClassifier>();
}

void TrafficLightClassifierNode::imageRoiCallback(const sensor_msgs::ImageConstPtr &input_image_msg,
                                                  const autoware_traffic_light_msgs::TrafficLightRoiArrayConstPtr &input_rois_msg)
{
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
  }

  autoware_traffic_light_msgs::TrafficLightStateArray output_msg;

  for (size_t i = 0; i < input_rois_msg->rois.size(); ++i)
  {

    const sensor_msgs::RegionOfInterest &roi = input_rois_msg->rois.at(i).roi;
    cv::Mat cliped_image(cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));

    std::vector<autoware_traffic_light_msgs::LampState> lamp_states;

    if (!classifier_ptr_->getLampState(cliped_image, lamp_states))
    {
      ROS_ERROR("failed classify image, abort callback");
      return;
    }
    autoware_traffic_light_msgs::TrafficLightState tl_state;
    tl_state.id = input_rois_msg->rois.at(i).id;
    tl_state.lamp_states = lamp_states;
    output_msg.states.push_back(tl_state);
  }

  output_msg.header = input_image_msg->header;
  tl_states_pub_.publish(output_msg);
}

} // namespace traffic_light
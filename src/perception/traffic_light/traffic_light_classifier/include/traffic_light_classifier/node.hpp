#pragma once

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/subscriber_filter.h>
#include <image_transport/image_transport.h>
#include <autoware_traffic_light_msgs/TrafficLightRoiArray.h>
#include <autoware_traffic_light_msgs/TrafficLightStateArray.h>
#include <autoware_traffic_light_msgs/TrafficLightState.h>
#include <autoware_traffic_light_msgs/LampState.h>
#include <cv_bridge/cv_bridge.h>

#include <traffic_light_classifier/classifier_interface.hpp>
#include <traffic_light_classifier/color_classifier.hpp>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <memory>

namespace traffic_light
{
class TrafficLightClassifierNode
{
public:
  TrafficLightClassifierNode();
  virtual ~TrafficLightClassifierNode(){};

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<autoware_traffic_light_msgs::TrafficLightRoiArray> roi_sub_;
  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
                                                    autoware_traffic_light_msgs::TrafficLightRoiArray>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                    autoware_traffic_light_msgs::TrafficLightRoiArray>
      ApproximateSyncPolicy;
  typedef message_filters::Synchronizer<ApproximateSyncPolicy> ApproximateSync;
  ApproximateSync approximate_sync_;
  bool is_approximate_sync_;
  ros::Publisher tl_states_pub_;
  std::shared_ptr<ClassifierInterface> classifier_ptr_;
  void imageRoiCallback(const sensor_msgs::ImageConstPtr &input_image_msg,
                        const autoware_traffic_light_msgs::TrafficLightRoiArrayConstPtr &input_rois_msg);
};

} // namespace traffic_light

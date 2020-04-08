#pragma once
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include "autoware_traffic_light_msgs/TrafficLightRoiArray.h"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "sensor_msgs/Image.h"

#include <memory>
#include <string>

namespace traffic_light {
class TrafficLightRoiImageSaver {
 public:
  TrafficLightRoiImageSaver();
  virtual ~TrafficLightRoiImageSaver();

 private:
  void imageRoiCallback(const sensor_msgs::ImageConstPtr& input_image_msg,
                        const autoware_traffic_light_msgs::TrafficLightRoiArray::ConstPtr& input_tl_roi_msg);

  ros::NodeHandle nh_, pnh_;
  image_transport::ImageTransport image_transport_;
  image_transport::SubscriberFilter image_sub_;
  message_filters::Subscriber<autoware_traffic_light_msgs::TrafficLightRoiArray> roi_sub_;
  image_transport::Publisher image_pub_;
  std::string save_dir_;
  std::shared_ptr<ros::Rate> save_rate_ptr_;
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                                                          autoware_traffic_light_msgs::TrafficLightRoiArray>
      SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Sync;
  Sync sync_;
};

}  // namespace traffic_light
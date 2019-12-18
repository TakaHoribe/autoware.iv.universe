#ifndef _TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_BBOX_
#define _TRAFFICLIGHT_RECOGNIZER_FEAT_PROJ_BBOX_

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <autoware_msgs/Signals.h>
#include <autoware_traffic_light_msgs/StampedRoi.h>
#include <sensor_msgs/RegionOfInterest.h>

#include "new_trafficlight_recognizer/utils.h"

namespace new_trafficlight_recognizer
{
  class FeatProjBBox : public nodelet::Nodelet
  {
  public:

    float z_max = 100;

    float z_min = 20;

    int w_max = 350;

    int w_min = 50;

    int h_max = 200;

    int h_min = 50;

    void onInit();

    void callback(const sensor_msgs::Image::ConstPtr& image_msg,
                  const autoware_msgs::Signals::ConstPtr& signal_msg);

    bool extract(const cv::Size& image_size,
                 const std::vector<autoware_msgs::ExtractedPosition>& signals,
                 autoware_traffic_light_msgs::StampedRoi& projected_rois);

    cv::Size calc_projected_area(float z);

  private:

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    ros::Publisher projected_rois_pub_;

    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      autoware_msgs::Signals
      > SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      autoware_msgs::Signals
      > ApproximateSyncPolicy;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;

    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;

    message_filters::Subscriber<sensor_msgs::Image> sub_image_;

    message_filters::Subscriber<autoware_msgs::Signals> sub_signal_;

    std::shared_ptr<Utils> utils_;

    bool debug_ = true;

    bool is_approximate_sync_ = true;

  }; // class FeatProjBBox

} // namespace new_trafficlight_recognizer

#endif

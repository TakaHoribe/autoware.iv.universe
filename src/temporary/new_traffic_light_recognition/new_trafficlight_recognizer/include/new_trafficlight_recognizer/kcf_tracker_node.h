#ifndef _TRAFFICLIGHT_RECOGNIZERR_KCF_TRACKER_NODE_
#define _TRAFFICLIGHT_RECOGNIZERR_KCF_TRACKER_NODE_

#include <stdlib.h>
#include <time.h>

#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "new_trafficlight_recognizer/kcf_tracker.h"
#include "new_trafficlight_recognizer/utils.h"

namespace new_trafficlight_recognizer
{

  class KcfTrackerNode : public nodelet::Nodelet
  {
  public:

    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      autoware_traffic_light_msgs::StampedRoi
      > SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      autoware_traffic_light_msgs::StampedRoi
      > ApproximateSyncPolicy;

    typedef std::shared_ptr<Utils> UtilsPtr;

    typedef std::shared_ptr<MultiKcfTracker> MultiKcfTrackerPtr;

    bool debug_log_ = false;

    double kernel_sigma_ = 0.5;

    double cell_size_ = 4;

    double num_scales_ = 7;

    bool is_approximate_sync_ = true;

    int buffer_size_ = 100;

    int interpolation_frequency_ = 1;

    double detected_boxes_stamp_ = 0;

    std::vector<cv::Rect> detected_boxes_;

    std::vector<int> detected_boxes_signals_;

    ros::NodeHandle nh_;

    ros::NodeHandle pnh_;

    ros::Publisher output_rois_pub_;

    ros::Publisher output_debug_pub_;

    ros::Subscriber boxes_sub;

    boost::mutex mutex_;

    UtilsPtr utils_;

    MultiKcfTrackerPtr multi_kcf_tracker_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;

    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_;

    message_filters::Subscriber<autoware_traffic_light_msgs::StampedRoi> stamped_roi_sub_;

    void onInit();

    void boxes_callback(const autoware_traffic_light_msgs::StampedRoi::ConstPtr& boxes);

    void callback(const sensor_msgs::Image::ConstPtr& image_msg,
                  const autoware_traffic_light_msgs::StampedRoi::ConstPtr& stamped_roi_msg);

  }; // KcfTrackerNode


} // namespace new_trafficlight_recognizer

#endif

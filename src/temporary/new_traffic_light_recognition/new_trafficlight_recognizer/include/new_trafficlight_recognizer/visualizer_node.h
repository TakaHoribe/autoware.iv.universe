#ifndef TRAFFICLIGHT_RECOGNIZER_VISUALIZER_H
#define TRAFFICLIGHT_RECOGNIZER_VISUALIZER_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/Point.h>
#include <autoware_traffic_light_msgs/StampedRoi.h>
#include <autoware_msgs/Signals.h>
#include <jsk_recognition_msgs/BoundingBox.h>
#include <jsk_recognition_msgs/BoundingBoxArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "new_trafficlight_recognizer/utils.h"

namespace new_trafficlight_recognizer
{

  class TrafficLightVisualizerNode
  {

  public:

    typedef std::shared_ptr<Utils> UtilsPtr;

    typedef message_filters::sync_policies::ExactTime<
      sensor_msgs::Image,
      autoware_traffic_light_msgs::StampedRoi,
      autoware_traffic_light_msgs::StampedRoi,
      autoware_msgs::Signals,
      autoware_traffic_light_msgs::TrafficLightStateArray
      > SyncPolicy;

    typedef message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image,
      autoware_traffic_light_msgs::StampedRoi,
      autoware_traffic_light_msgs::StampedRoi,
      autoware_msgs::Signals,
      autoware_traffic_light_msgs::TrafficLightStateArray
      > ApproximateSyncPolicy;


    ros::NodeHandle nh_;

    boost::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;

    boost::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;

    message_filters::Subscriber<sensor_msgs::Image> image_sub_;

    message_filters::Subscriber<autoware_traffic_light_msgs::StampedRoi> projected_roi_sub_;

    message_filters::Subscriber<autoware_traffic_light_msgs::StampedRoi> tracker_roi_sub_;

    message_filters::Subscriber<autoware_msgs::Signals> signals_sub_;

    message_filters::Subscriber<autoware_traffic_light_msgs::TrafficLightStateArray> light_state_sub_;


    bool is_approximate_sync_ = true;

    UtilsPtr utils_;

    ros::Publisher visualization_image_pub_;

    ros::Publisher visualization_lines_pub_;

    ros::Publisher visualization_bboxes_pub_;


    void callback
      (const sensor_msgs::Image::ConstPtr& image_msg,
       const autoware_traffic_light_msgs::StampedRoi::ConstPtr& projected_roi_msg,
       const autoware_traffic_light_msgs::StampedRoi::ConstPtr& tracker_roi_msg,
       const autoware_msgs::Signals::ConstPtr& signals_msg,
       const autoware_traffic_light_msgs::TrafficLightStateArray::ConstPtr& light_state_msg);

    void run();

  private:

  }; // TrafficLightVisualizerNode

} // new_trafficlight_recognizer

#endif

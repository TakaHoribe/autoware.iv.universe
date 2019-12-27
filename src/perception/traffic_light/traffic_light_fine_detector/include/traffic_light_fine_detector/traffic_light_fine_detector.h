#ifndef TRAFFIC_LIGHT_DETECTOR_H
#define TRAFFIC_LIGHT_DETECTOR_H

#include <string>
#include <memory>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv/cv.hpp>

#include "TrtNet.h"
#include "data_reader.h"

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Header.h>
#include <cv_bridge/cv_bridge.h>

#include <autoware_traffic_light_msgs/TrafficLightRoiArray.h>

typedef struct Detection
{
  float x, y, w, h, prob;
} Detection;

namespace traffic_light
{
class TrafficLightFineDetectorNode
{
private:
  // variables

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, autoware_traffic_light_msgs::TrafficLightRoiArray>
      SyncPolicy;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, autoware_traffic_light_msgs::TrafficLightRoiArray>
      ApproximateSyncPolicy;

  std::shared_ptr<message_filters::Synchronizer<SyncPolicy> > sync_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy> > approximate_sync_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  message_filters::Subscriber<autoware_traffic_light_msgs::TrafficLightRoiArray> traffic_light_roi_sub_;
  ros::Publisher output_rois_pub_;

  bool is_approximate_sync_;
  double score_thresh_;

  std::unique_ptr<Tn::trtNet> net_ptr_;
  std::vector<std::vector<int> > output_shape = { { 1, 18, 8, 8 }, { 1, 18, 16, 16 }, { 1, 18, 32, 32 } };
  std::vector<std::vector<int> > g_masks = { { 6, 7, 8 }, { 3, 4, 5 }, { 0, 1, 2 } };
  std::vector<std::vector<int> > g_anchors = { { 72, 35 },  { 53, 58 },  { 127, 43 }, { 90, 62 },  { 61, 104 },
                                               { 141, 70 }, { 105, 95 }, { 95, 154 }, { 157, 135 } };

  // functions
  void initROS();

  void callback(const sensor_msgs::Image::ConstPtr& image_msg,
                const autoware_traffic_light_msgs::TrafficLightRoiArray::ConstPtr& traffic_light_roi_msg);

  inline float sigmoid(float in)
  {
    return 1.f / (1.f + std::exp(-in));
  }

  std::vector<float> prepareImage(cv::Mat& in_img);
  std::vector<Tn::Bbox> postProcessImg(float* output, const int classes, cv::Mat& img);
  void doNms(std::vector<Detection>& detections, float nms_thresh);
  bool rosmsg2cvmat(const sensor_msgs::Image::ConstPtr& image_msg, cv::Mat& image);
  bool fit_in_frame(cv::Point& lt, cv::Point& rb, const cv::Size& size);
  void cvrect2tlroimsg(const cv::Rect& rect, const int32_t id, autoware_traffic_light_msgs::TrafficLightRoi& tl_roi);
public:
  TrafficLightFineDetectorNode();
  ~TrafficLightFineDetectorNode();
  void run();

};  // TrafficLightFineDetector

}  // namespace

#endif

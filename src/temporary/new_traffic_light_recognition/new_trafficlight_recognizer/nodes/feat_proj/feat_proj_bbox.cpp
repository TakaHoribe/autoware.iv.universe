#include "new_trafficlight_recognizer/feat_proj_bbox.h"

namespace new_trafficlight_recognizer
{
  void FeatProjBBox::onInit()
  {
    nh_ = getNodeHandle();
    pnh_ = getPrivateNodeHandle();

    pnh_.getParam("z_max", z_max);
    pnh_.getParam("z_min", z_min);
    pnh_.getParam("w_max", w_max);
    pnh_.getParam("w_min", w_min);
    pnh_.getParam("h_max", h_max);
    pnh_.getParam("h_min", h_min);
    pnh_.getParam("approximate_sync", is_approximate_sync_);

    projected_rois_pub_ = pnh_.advertise<autoware_traffic_light_msgs::StampedRoi>("output_projected_roi", 1);

    sub_image_.subscribe(pnh_, "input_image", 1);
    sub_signal_.subscribe(pnh_, "input_signal", 1);

    utils_ = std::make_shared<Utils>();

    if (is_approximate_sync_){
      approximate_sync_ =
        boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(1000);
      approximate_sync_->connectInput(sub_image_, sub_signal_);
      approximate_sync_->registerCallback(boost::bind(&FeatProjBBox::callback, this, _1, _2));
    } else {
      sync_  = boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(1000);
      sync_->connectInput(sub_image_, sub_signal_);
      sync_->registerCallback(boost::bind(&FeatProjBBox::callback, this, _1, _2));
    }
  }

  cv::Size FeatProjBBox::calc_projected_area(float z)
  {
    cv::Size projected_area;

    // remove outlier
    if (z > z_max) z = z_max;
    if (z < z_min) z = z_min;

    projected_area.width = w_min + (z_max - z) * (w_max - w_min) / (z_max - z_min);
    projected_area.height = h_min + (z_max - z) * (h_max - h_min) / (z_max - z_min);

    return projected_area;
  }

  bool FeatProjBBox::extract(const cv::Size& image_size,
                             const std::vector<autoware_msgs::ExtractedPosition>& signals,
                             autoware_traffic_light_msgs::StampedRoi& projected_rois)
  {
    if (image_size.width == 0 || image_size.height == 0)
      {
        ROS_ERROR("image_size is invalid value size[%d, %d]",
                  image_size.width, image_size.height);
        return false;
      }

    for (auto signal : signals) {

      int u = static_cast<int>(signal.u);
      int v = static_cast<int>(signal.v);
      int z = static_cast<float>(signal.z);
      if (u < 0 || u > image_size.width || v < 0 || v > image_size.height)
        continue;

      cv::Size projected_area = calc_projected_area(z);
      cv::Point lt = cv::Point(u - projected_area.width * 0.5,
                               v - projected_area.height * 0.5);
      cv::Point rb = cv::Point(u + projected_area.width * 0.5,
                               v + projected_area.height * 0.5);

      if ( !utils_->fit_in_frame(lt, rb, image_size) )
        {
          ROS_ERROR("failed to fit in frame");
        return false;
        }

      sensor_msgs::RegionOfInterest roi_rect;
      roi_rect.x_offset = lt.x;
      roi_rect.y_offset = lt.y;
      roi_rect.width = rb.x - lt.x;
      roi_rect.height = rb.y - lt.y;
      projected_rois.roi_array.push_back(roi_rect);
      projected_rois.signals.push_back(static_cast<int>(signal.signalId));
    }

    return true;
  }

  void FeatProjBBox::callback(const sensor_msgs::Image::ConstPtr& image_msg,
                              const autoware_msgs::Signals::ConstPtr& signal_msg)
  {
    cv::Mat image;
    utils_->rosmsg2cvmat(image_msg, image);

    autoware_traffic_light_msgs::StampedRoi projected_rois;
    extract(image.size(), signal_msg->Signals, projected_rois);

    projected_rois.header = image_msg->header;
    projected_rois_pub_.publish(projected_rois);
  }

} // namespace new_trafficlight_recognizer

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(new_trafficlight_recognizer::FeatProjBBox, nodelet::Nodelet)

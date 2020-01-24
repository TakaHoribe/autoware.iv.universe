#include "node.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string>

namespace traffic_light
{

TrafficLightRoiImageSaver::TrafficLightRoiImageSaver() : nh_(""),
                                                         pnh_("~"),
                                                         image_transport_(pnh_),
                                                         image_sub_(image_transport_, "input/image", 1),
                                                         roi_sub_(pnh_, "input/rois", 1),
                                                         sync_(SyncPolicy(10), image_sub_, roi_sub_)
{
    image_pub_ = image_transport_.advertise("output/image", 1);
    sync_.registerCallback(boost::bind(&TrafficLightRoiImageSaver::imageRoiCallback, this, _1, _2));
    
    pnh_.getParam("save_dir", save_dir_);
}
TrafficLightRoiImageSaver::~TrafficLightRoiImageSaver()
{
}
void TrafficLightRoiImageSaver::imageRoiCallback(const sensor_msgs::ImageConstPtr &input_image_msg,
                                                 const autoware_traffic_light_msgs::TrafficLightRoiArrayConstPtr &input_tl_roi_msg)
{
    ros::Time current_time = ros::Time::now();
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(input_image_msg, sensor_msgs::image_encodings::BGR8);
        for (size_t i = 0; i < input_tl_roi_msg->rois.size(); ++i)
        {
            const sensor_msgs::RegionOfInterest &roi = input_tl_roi_msg->rois.at(i).roi;
            cv::Mat cliped_image(cv_ptr->image, cv::Rect(roi.x_offset, roi.y_offset, roi.width, roi.height));
            std::stringstream save_fine_name_stream;
            save_fine_name_stream << "/" << input_tl_roi_msg->rois.at(i).id << "_" << current_time.toSec() << ".png";
            std::string save_fine_name;
            save_fine_name_stream >> save_fine_name;
            cv::imwrite(save_fine_name, cliped_image);
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", input_image_msg->encoding.c_str());
    }
    image_pub_.publish(cv_ptr->toImageMsg());
}
} // namespace traffic_light
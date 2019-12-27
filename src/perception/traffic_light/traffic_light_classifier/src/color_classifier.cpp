#include "traffic_light_classifier/color_classifier.hpp"

namespace traffic_light
{
ColorClassifier::ColorClassifier()
    : nh_(""), pnh_("~"), ratio_threshold_(0.02)
{
    std::vector<int> green_max_hsv;
    std::vector<int> green_min_hsv;
    std::vector<int> red_max_hsv;
    std::vector<int> red_min_hsv;
    std::vector<int> yellow_max_hsv;
    std::vector<int> yellow_min_hsv;

    pnh_.getParam("green_max", green_max_hsv);
    pnh_.getParam("green_min", green_min_hsv);
    pnh_.getParam("yellow_max", yellow_max_hsv);
    pnh_.getParam("yellow_min", yellow_min_hsv);
    pnh_.getParam("red_max", red_max_hsv);
    pnh_.getParam("red_min", red_min_hsv);

    min_hsv_green_ = cv::Scalar(green_min_hsv.at(ColorClassifier::HSV::Hue),
                                green_min_hsv.at(ColorClassifier::HSV::Sat),
                                green_min_hsv.at(ColorClassifier::HSV::Val));
    max_hsv_green_ = cv::Scalar(green_max_hsv.at(ColorClassifier::HSV::Hue),
                                green_max_hsv.at(ColorClassifier::HSV::Sat),
                                green_max_hsv.at(ColorClassifier::HSV::Val));

    min_hsv_yellow_ = cv::Scalar(yellow_min_hsv.at(ColorClassifier::HSV::Hue),
                                 yellow_min_hsv.at(ColorClassifier::HSV::Sat),
                                 yellow_min_hsv.at(ColorClassifier::HSV::Val));
    max_hsv_yellow_ = cv::Scalar(yellow_max_hsv.at(ColorClassifier::HSV::Hue),
                                 yellow_max_hsv.at(ColorClassifier::HSV::Sat),
                                 yellow_max_hsv.at(ColorClassifier::HSV::Val));

    min_hsv_red_ = cv::Scalar(red_min_hsv.at(ColorClassifier::HSV::Hue),
                              red_min_hsv.at(ColorClassifier::HSV::Sat),
                              red_min_hsv.at(ColorClassifier::HSV::Val));
    max_hsv_red_ = cv::Scalar(red_max_hsv.at(ColorClassifier::HSV::Hue),
                              red_max_hsv.at(ColorClassifier::HSV::Sat),
                              red_max_hsv.at(ColorClassifier::HSV::Val));
#if 0
    cv::namedWindow("raw");
    cv::namedWindow("debug");
    // cv::namedWindow("debug_green");
    // cv::namedWindow("debug_yellow");
    // cv::namedWindow("debug_red");
    // cv::namedWindow("green_hsv");
    // cv::namedWindow("yellow_hsv");
    // cv::namedWindow("red_hsv");
    // cv::namedWindow("green_bin");
    // cv::namedWindow("yellow_bin");
    // cv::namedWindow("red_bin");
    // cv::namedWindow("green_filtered_bin");
    // cv::namedWindow("yellow_filtered_bin");
    // cv::namedWindow("red_filtered_bin");
#endif
}

bool ColorClassifier::getLampState(const cv::Mat &input_image, std::vector<autoware_traffic_light_msgs::LampState> &states)
{
    cv::Mat green_image;
    cv::Mat yellow_image;
    cv::Mat red_image;
    filterHSV(input_image, green_image, yellow_image, red_image);
    // binalize
    cv::Mat green_bin_image;
    cv::Mat yellow_bin_image;
    cv::Mat red_bin_image;
    const int bin_threshold = 127;
    cv::threshold(green_image, green_bin_image, bin_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(yellow_image, yellow_bin_image, bin_threshold, 255, cv::THRESH_BINARY);
    cv::threshold(red_image, red_bin_image, bin_threshold, 255, cv::THRESH_BINARY);
    // filter noise
    cv::Mat green_filtered_bin_image;
    cv::Mat yellow_filtered_bin_image;
    cv::Mat red_filtered_bin_image;
    cv::Mat element4 = (cv::Mat_<uchar>(3, 3) << 0, 1, 0, 1, 1, 1, 0, 1, 0);
    cv::erode(green_bin_image, green_filtered_bin_image, element4, cv::Point(-1,-1), 1);
    cv::erode(yellow_bin_image, yellow_filtered_bin_image, element4, cv::Point(-1,-1), 1);
    cv::erode(red_bin_image, red_filtered_bin_image, element4, cv::Point(-1,-1), 1);
    cv::dilate(green_filtered_bin_image, green_filtered_bin_image, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(yellow_filtered_bin_image, yellow_filtered_bin_image, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(red_filtered_bin_image, red_filtered_bin_image, cv::Mat(), cv::Point(-1,-1), 1);

    /* debug */
#if 0
    cv::Mat debug_green_image;
    cv::Mat debug_yellow_image;
    cv::Mat debug_red_image;
    cv::hconcat(green_image, green_bin_image, debug_green_image);
    cv::hconcat(debug_green_image, green_filtered_bin_image, debug_green_image);
    cv::hconcat(yellow_image, yellow_bin_image, debug_yellow_image);
    cv::hconcat(debug_yellow_image, yellow_filtered_bin_image, debug_yellow_image);
    cv::hconcat(red_image, red_bin_image, debug_red_image);
    cv::hconcat(debug_red_image, red_filtered_bin_image, debug_red_image);
    cv::Mat debug_image;
    cv::rectangle(debug_green_image, cv::Point(0, 0),
                  cv::Point(debug_green_image.cols-1, debug_green_image.rows-1),
                  cv::Scalar(0, 0, 0), 1, CV_AA, 0);
    cv::rectangle(debug_yellow_image, cv::Point(0, 0),
                  cv::Point(debug_yellow_image.cols-1, debug_yellow_image.rows-1),
                  cv::Scalar(0, 0, 0), 1, CV_AA, 0);
    cv::rectangle(debug_red_image, cv::Point(0, 0),
                  cv::Point(debug_red_image.cols-1, debug_red_image.rows-1),
                  cv::Scalar(0, 0, 0), 1, CV_AA, 0);
    cv::vconcat(debug_green_image, debug_yellow_image, debug_image);
    cv::vconcat(debug_image, debug_red_image, debug_image);

    cv::imshow("raw", input_image);
    cv::imshow("debug", debug_image);
    // cv::imshow("debug_green", debug_green_image);
    // cv::imshow("debug_yellow", debug_yellow_image);
    // cv::imshow("debug_red", debug_red_image);
    // cv::imshow("green_hsv", green_image);
    // cv::imshow("yellow_hsv", yellow_image);
    // cv::imshow("red_hsv", red_image);
    // cv::imshow("green_bin", green_bin_image);
    // cv::imshow("yellow_bin", yellow_bin_image);
    // cv::imshow("red_bin", red_bin_image);
    // cv::imshow("green_filtered_bin", green_filtered_bin_image);
    // cv::imshow("yellow_filtered_bin", yellow_filtered_bin_image);
    // cv::imshow("red_filtered_bin", red_filtered_bin_image);
    cv::waitKey(0);
    #endif
    /* --- */

    const int green_pixel_num = cv::countNonZero(green_bin_image);
    const int yellow_pixel_num = cv::countNonZero(yellow_bin_image);
    const int red_pixel_num = cv::countNonZero(red_bin_image);
    const double green_ratio = (double)green_pixel_num / (double)(green_bin_image.rows * green_bin_image.cols);
    const double yellow_ratio = (double)yellow_pixel_num / (double)(yellow_bin_image.rows * yellow_bin_image.cols);
    const double red_ratio = (double)red_pixel_num / (double)(red_bin_image.rows * red_bin_image.cols);
    if (yellow_ratio < green_ratio && red_ratio < green_ratio)
    {
        autoware_traffic_light_msgs::LampState state;
        state.type = autoware_traffic_light_msgs::LampState::GREEN;
        state.confidence = 1.0;
        states.push_back(state);
    }
    else if (green_ratio < yellow_ratio && red_ratio < yellow_ratio)
    {
        autoware_traffic_light_msgs::LampState state;
        state.type = autoware_traffic_light_msgs::LampState::YELLOW;
        state.confidence = 1.0;
        states.push_back(state);
    }
    else if (green_ratio < red_ratio && yellow_ratio < red_ratio)
    {
        autoware_traffic_light_msgs::LampState state;
        state.type = autoware_traffic_light_msgs::LampState::RED;
        state.confidence = 1.0;
        states.push_back(state);
    }
    return true;
}

bool ColorClassifier::filterHSV(const cv::Mat &input_image,
                                cv::Mat &green_image,
                                cv::Mat &yellow_image,
                                cv::Mat &red_image)
{
    cv::Mat hsv_image;
    cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);
    try
    {
        cv::inRange(hsv_image, min_hsv_green_, max_hsv_green_, green_image);
        cv::inRange(hsv_image, min_hsv_yellow_, max_hsv_yellow_, yellow_image);
        cv::inRange(hsv_image, min_hsv_red_, max_hsv_red_, red_image);
    }
    catch (cv::Exception &e)
    {
        ROS_ERROR("failed to filter image by hsv value : %s", e.what());
        return false;
    }
    return true;
}
} // namespace traffic_light

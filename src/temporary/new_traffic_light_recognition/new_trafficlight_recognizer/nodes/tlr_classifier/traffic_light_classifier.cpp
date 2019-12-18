#include "new_trafficlight_recognizer/traffic_light_classifier.h"

namespace new_trafficlight_recognizer
{

  ColorClassifier::ColorClassifier()
  {
    utils_ = std::make_shared<Utils>();
  }

  bool ColorClassifier::hsv_filter(const cv::Mat &input_image,
                                   std::vector<cv::Mat> &output_images,
                                   std::vector<cv::Mat>& debug_images)
  {
    for (int i=0; i<lower_ranges_.size(); ++i)
      {
        cv::Mat hsv_image;
        cv::cvtColor(input_image, hsv_image, cv::COLOR_BGR2HSV);

        cv::Mat dst_image;
        try
          {
            cv::inRange(hsv_image, lower_ranges_.at(i), upper_ranges_.at(i), dst_image);
          }
        catch (cv::Exception& e)
          {
            ROS_ERROR("failed to filter image by hsv value : %s", e.what());
            return false;
          }

        output_images.push_back(dst_image);

        cv::cvtColor(dst_image, dst_image, CV_GRAY2BGR);
        debug_images.push_back(dst_image);

      }

    return true;
  }


  bool ColorClassifier::get_color_ratios(const std::vector<cv::Mat> &input_images,
                                         std::vector<float> &ratios)
  {
    for (int i=0; i<input_images.size(); ++i)
      {
        cv::Mat image = input_images.at(i).clone();
        int image_size = image.cols * image.rows;
        int area = utils_->get_area(image);
        float ratio = static_cast<float>(area) / static_cast<float>(image_size);
        // std::cerr << "ratio: " << ratio << std::endl;
        ratios.push_back(ratio);
      }

    return true;
  }


  bool ColorClassifier::get_lamp_states(autoware_traffic_light_msgs::LampStateArray& states,
                                        std::vector<float> ratios)
  {
    if (ratios.at(static_cast<int>(ColorClassifier::Lamp::GREEN)) > ratios_thresh_)
      {
        autoware_traffic_light_msgs::LampState state;
        state.type = autoware_traffic_light_msgs::LampState::GREEN;
        states.states.push_back(state);
      }
    if (ratios.at(static_cast<int>(ColorClassifier::Lamp::YELLOW)) > ratios_thresh_)
      {
        autoware_traffic_light_msgs::LampState state;
        state.type = autoware_traffic_light_msgs::LampState::YELLOW;
        states.states.push_back(state);
      }
    if (ratios.at(static_cast<int>(ColorClassifier::Lamp::RED)) > ratios_thresh_)
      {
        autoware_traffic_light_msgs::LampState state;
        state.type = autoware_traffic_light_msgs::LampState::RED;
        states.states.push_back(state);
      }

    return true;
  }

  bool ColorClassifier::get_params(const ros::NodeHandle& pnh)
  {
    pnh.getParam("buffer_size", buffer_size_);
    pnh.getParam("ratios_thresh", ratios_thresh_);

    std::vector<int> green_max_hsv;
    std::vector<int> green_min_hsv;
    std::vector<int> red_max_hsv;
    std::vector<int> red_min_hsv;
    std::vector<int> yellow_max_hsv;
    std::vector<int> yellow_min_hsv;

    pnh.getParam("green_max", green_max_hsv);
    pnh.getParam("green_min", green_min_hsv);
    pnh.getParam("yellow_max", yellow_max_hsv);
    pnh.getParam("yellow_min", yellow_min_hsv);
    pnh.getParam("red_max", red_max_hsv);
    pnh.getParam("red_min", red_min_hsv);

    lower_ranges_.push_back
      (cv::Scalar(green_min_hsv.at(ColorClassifier::HSV::Hue),
                  green_min_hsv.at(ColorClassifier::HSV::Sat),
                  green_min_hsv.at(ColorClassifier::HSV::Val)));
    upper_ranges_.push_back
      (cv::Scalar(green_max_hsv.at(ColorClassifier::HSV::Hue),
                  green_max_hsv.at(ColorClassifier::HSV::Sat),
                  green_max_hsv.at(ColorClassifier::HSV::Val)));

    lower_ranges_.push_back
      (cv::Scalar(yellow_min_hsv.at(ColorClassifier::HSV::Hue),
                  yellow_min_hsv.at(ColorClassifier::HSV::Sat),
                  yellow_min_hsv.at(ColorClassifier::HSV::Val)));
    upper_ranges_.push_back
      (cv::Scalar(yellow_max_hsv.at(ColorClassifier::HSV::Hue),
                  yellow_max_hsv.at(ColorClassifier::HSV::Sat),
                  yellow_max_hsv.at(ColorClassifier::HSV::Val)));

    lower_ranges_.push_back
      (cv::Scalar(red_min_hsv.at(ColorClassifier::HSV::Hue),
                  red_min_hsv.at(ColorClassifier::HSV::Sat),
                  red_min_hsv.at(ColorClassifier::HSV::Val)));
    upper_ranges_.push_back
      (cv::Scalar(red_max_hsv.at(ColorClassifier::HSV::Hue),
                  red_max_hsv.at(ColorClassifier::HSV::Sat),
                  red_max_hsv.at(ColorClassifier::HSV::Val)));


    if (lower_ranges_.size() != 3 || upper_ranges_.size() != 3)
      {
        ROS_ERROR("lower_ranges_ and upper_ranges_ size is not suitable");
        return false;
      }

    return true;
  }


  ArrowClassifier::ArrowClassifier()
  {
    utils_ = std::make_shared<Utils>();
  }

  bool ArrowClassifier::get_shape(const cv::Mat& input_image, std::vector<std::vector<cv::Point> >& contours)
  {
    cv::Mat image = input_image.clone();

    try
      {
        cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
                                                    cv::Size(2*dilation_size_+1, 2*dilation_size_+1),
                                                    cv::Point(dilation_size_, dilation_size_));
        cv::dilate(image, image, element);

        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(image, contours, hierarchy,
                         CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));
      }
    catch (cv::Exception& e)
      {
        ROS_ERROR("failed to get shape : %s", e.what());
        return false;
      }

    return true;
  }

  bool ArrowClassifier::calc_arrow_direction(const cv::Mat& image,
                                             const cv::Rect rect,
                                             autoware_traffic_light_msgs::LampState& state)
  {

    cv::Mat lt_cell = image(cv::Rect(rect.x,
                                     rect.y,
                                     static_cast<int>(rect.width * 0.5),
                                     static_cast<int>(rect.height * 0.5)));
    cv::Mat lb_cell = image(cv::Rect(rect.x,
                                     static_cast<int>(rect.y + rect.height * 0.5),
                                     static_cast<int>(rect.width * 0.5),
                                     static_cast<int>(rect.height * 0.5)));
    cv::Mat rt_cell = image(cv::Rect(static_cast<int>(rect.x + rect.width * 0.5),
                                     rect.y,
                                     static_cast<int>(rect.width * 0.5),
                                     static_cast<int>(rect.height * 0.5)));
    cv::Mat rb_cell = image(cv::Rect(static_cast<int>(rect.x + rect.width * 0.5),
                                     static_cast<int>(rect.y + rect.height * 0.5),
                                     static_cast<int>(rect.width * 0.5),
                                     static_cast<int>(rect.height * 0.5)));

    std::pair<Utils::Position, float> lt_area =
      std::make_pair(Utils::Position::LEFTTOP, utils_->get_area(lt_cell));
    std::pair<Utils::Position, float> lb_area =
      std::make_pair(Utils::Position::LEFTBOTTOM, utils_->get_area(lb_cell));
    std::pair<Utils::Position, float> rt_area =
      std::make_pair(Utils::Position::RIGHTTOP, utils_->get_area(rt_cell));
    std::pair<Utils::Position, float> rb_area =
      std::make_pair(Utils::Position::RIGHTBOTTOM, utils_->get_area(rb_cell));

    std::vector<std::pair<Utils::Position, float>> area_list{lt_area, lb_area, rt_area, rb_area};

    std::sort(area_list.begin(), area_list.end(),
              [](const std::pair<Utils::Position, float> &left,
                 const std::pair<Utils::Position, float> &right)
              {return left.second < right.second;});

    std::vector<Utils::Position> cells{area_list.at(0).first, area_list.at(1).first};

    if (utils_->in_vector(cells, Utils::Position::LEFTTOP) &&
        utils_->in_vector(cells, Utils::Position::RIGHTBOTTOM))
      {
        state.type = autoware_traffic_light_msgs::LampState::UP;
      }
    else if (utils_->in_vector(cells, Utils::Position::LEFTBOTTOM) &&
             utils_->in_vector(cells, Utils::Position::RIGHTBOTTOM))
      {
        state.type = autoware_traffic_light_msgs::LampState::DOWN;
      }
    else if (utils_->in_vector(cells, Utils::Position::LEFTTOP) &&
             utils_->in_vector(cells, Utils::Position::LEFTBOTTOM))
      {
        state.type = autoware_traffic_light_msgs::LampState::LEFT;
      }
    else if (utils_->in_vector(cells, Utils::Position::RIGHTTOP) &&
             utils_->in_vector(cells, Utils::Position::RIGHTBOTTOM))
      {
        state.type = autoware_traffic_light_msgs::LampState::RIGHT;
      }

    return true;
  }


  bool ArrowClassifier::load_template(const std::string path)
  {
    template_image_ = cv::imread(path);
    if (! template_image_.data)
      {
        ROS_ERROR("failed load template image");
        return false;
      }
    try
      {
        cv::cvtColor(template_image_, template_image_, CV_BGR2GRAY);
        cv::threshold(template_image_, template_image_, 127, 255, 0);
      }
    catch (cv::Exception& e)
      {
        ROS_ERROR("failed to convert template image : %s", e.what());
        return false;
      }

    return true;
  }

  bool ArrowClassifier::get_params(const ros::NodeHandle& pnh)
  {
    std::string template_path;
    pnh.getParam("template_path", template_path);


    if (! load_template(template_path))
      return false;

    return true;
  }


  bool ArrowClassifier::get_index_pairs
  (const std::vector< std::vector<cv::Point> >& image_contours,
   const std::vector< std::vector<cv::Point> >& template_contours,
   std::vector<std::pair<int, double>>& index_pairs,
   const cv::Size& image_size)
  {
    for (int i=0; i<image_contours.size(); ++i)
      {
        for (int j=0; j<template_contours.size(); ++j)
          {

            double likelihood = 0;
            double area = 0;
            try
              {
                // matchShapes function does not work well
                // need debug
                likelihood = cv::matchShapes(image_contours.at(i),
                                             template_contours.at(j),
                                             CV_CONTOURS_MATCH_I1, 0.0);
              }
            catch (const cv::Exception& e)
              {
                std::cerr << "failed match shape: " << e.what() << std::endl;
                return false;
              }

            area = cv::contourArea(image_contours);
            if (likelihood < pair_thresh_ && area > area_thresh_)
              {
                std::pair<int, double> index_pair = std::make_pair(i, likelihood);
                index_pairs.push_back(index_pair);
              }
          }

      }

    std::sort(index_pairs.begin(), index_pairs.end(),
              [](const std::pair<int, double> &left, const std::pair<int, double> &right)
              {return left.second < right.second;});

    return true;
  }

  bool ArrowClassifier::get_arrow_rects
  (const std::vector< std::vector<cv::Point> >& image_contours,
   const std::vector< std::vector<cv::Point> >& template_contours,
   const cv::Mat& image,
   std::vector<cv::Rect>& rects,
   cv::Mat& debug_image)
  {
    std::vector<std::pair<int, double>> index_pairs;
    get_index_pairs(image_contours, template_contours, index_pairs, image.size());

    debug_image = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
    for (int i=0; i<index_pairs.size(); ++i)
      {
        int idx = index_pairs.at(i).first;
        cv::drawContours(debug_image, image_contours, idx, cv::Scalar(0,0,255), 2);
        cv::Rect rect = cv::boundingRect(image_contours.at(idx));
        rects.push_back(rect);
      }

    return true;
  }

  bool TrafficLightClassifierNode::generate_hsv_debug_image
  (const std::vector<cv::Mat>& hsv_debug_images,
   cv::Mat& hsv_debug_image)
  {
    cv::Mat dst_img;
    cv::hconcat(hsv_debug_images.at(0), hsv_debug_images.at(1), dst_img);
    cv::hconcat(dst_img, hsv_debug_images.at(2), dst_img);

    cv::Mat resized_image;
    cv::resize(dst_img, resized_image, cv::Size(), 0.333, 1, CV_INTER_AREA);
    hsv_debug_image = resized_image.clone();
  }

  bool TrafficLightClassifierNode::classify
  (const cv::Mat& image,
   autoware_traffic_light_msgs::LampStateArray& lamp_state_array,
   cv::Mat& color_debug_image,
   cv::Mat& arrow_debug_image,
   const cv::Rect& projected_roi)
  {
    ///// color classification /////

    std::vector<cv::Mat> filtered_images;
    std::vector<cv::Mat> hsv_images;
    if ( !color_classifier_->hsv_filter(image, filtered_images, hsv_images) )
      {
        return false;
      }

    // generate debug image for visualize
    cv::Mat hsv_debug_image;
    generate_hsv_debug_image(hsv_images, hsv_debug_image);

    cv::Point lt(projected_roi.x, projected_roi.y);
    cv::Point rb(projected_roi.x + hsv_debug_image.cols,
                 projected_roi.y + hsv_debug_image.rows);
    utils_->fit_in_frame(lt, rb, color_debug_image.size());

    cv::Mat transform = color_debug_image
      (cv::Rect(lt.x, lt.y, rb.x - lt.x, rb.y - lt.y));
    hsv_debug_image.copyTo(transform);

    std::vector<float> ratios;
    if( !color_classifier_->get_color_ratios(filtered_images, ratios) )
      {
        return false;
      }

    autoware_traffic_light_msgs::LampStateArray color_lamp_states;
    color_classifier_->get_lamp_states(color_lamp_states, ratios);
    if ( color_lamp_states.states.empty() )
      {
        autoware_traffic_light_msgs::LampState state;
        state.type = autoware_traffic_light_msgs::LampState::UNKNOWN;
        color_lamp_states.states.push_back(state);
      }


    ////// arrow classification /////

    cv::Mat gray_image = hsv_images.at(static_cast<int>(ColorClassifier::Lamp::GREEN)).clone();
    cv::cvtColor(gray_image, gray_image, CV_BGR2GRAY);
    std::vector< std::vector<cv::Point> > image_contours;
    if ( !arrow_classifier_->get_shape(gray_image, image_contours) )
      {
        return false;
      }

    std::vector< std::vector<cv::Point> > template_contours;
    if ( !arrow_classifier_->get_shape(arrow_classifier_->template_image_, template_contours) )
      {
        return false;
      }

    // debug contours
    cv::Mat dst_contour_img = cv::Mat::zeros(image.rows, image.cols, CV_8UC3);
    cv::drawContours(dst_contour_img, image_contours, -1, cv::Scalar(0,0,255), 3);

    // paste

    cv::Point arrow_lt(projected_roi.x, projected_roi.y);
    cv::Point arrow_rb(projected_roi.x + dst_contour_img.cols,
                       projected_roi.y + dst_contour_img.rows);
    utils_->fit_in_frame(arrow_lt, arrow_rb, arrow_debug_image.size());

    cv::Mat arrow_transform = arrow_debug_image
      (cv::Rect(arrow_lt.x, arrow_lt.y, arrow_rb.x - arrow_lt.x, arrow_rb.y - arrow_lt.y));
    dst_contour_img.copyTo(arrow_transform);

    autoware_traffic_light_msgs::LampStateArray arrow_lamp_states;
    std::vector<cv::Rect> rects;

    /* Commented by Taichi Higashide 2019/10/3
    ///////////////////////////////////////////////////////////////////////////
    [need debug]
    matchShapes opencv method in get_arrow_rects function does not work well

    arrow_classifier_->get_arrow_rects
      (image_contours, template_contours, image, rects, dst_contour_img);

    *//////////////////////////////////////////////////////////////////////////

    for (int i=0; i<rects.size(); ++i)
      {
        autoware_traffic_light_msgs::LampState state;
        arrow_classifier_->calc_arrow_direction(gray_image, rects.at(i), state);
        arrow_lamp_states.states.push_back(state);
      }

    utils_->merge_msg(color_lamp_states, arrow_lamp_states, lamp_state_array);

    return true;
  }

  void TrafficLightClassifierNode::callback
  (const sensor_msgs::Image::ConstPtr& image_msg,
   const autoware_traffic_light_msgs::StampedRoi::ConstPtr& stamped_roi_msg)
  {
    autoware_traffic_light_msgs::TrafficLightState traffilight_state_msg;

    cv::Mat image;
    utils_->rosmsg2cvmat(image_msg, image);

    autoware_traffic_light_msgs::TrafficLightStateArray trafficlight_state_array;
    cv::Mat concatenated_debug_image;
    cv::Mat color_debug_image = cv::Mat(image.size(), CV_8UC3, cv::Scalar(0,0,0));
    cv::Mat arrow_debug_image = cv::Mat(image.size(), CV_8UC3, cv::Scalar(0,0,0));

    for (int i=0; i<stamped_roi_msg->roi_array.size(); ++i)
      {
        autoware_traffic_light_msgs::TrafficLightState trafficlight_state;
        trafficlight_state.roi = stamped_roi_msg->roi_array.at(i);
        trafficlight_state.signal = stamped_roi_msg->signals.at(i);

        sensor_msgs::RegionOfInterest region_of_interest = stamped_roi_msg->roi_array.at(i);
        cv::Point lt(region_of_interest.x_offset, region_of_interest.y_offset);
        cv::Point rb(region_of_interest.x_offset + region_of_interest.width,
                     region_of_interest.y_offset + region_of_interest.height);
        utils_->fit_in_frame(lt, rb, image.size());

        cv::Rect roi = cv::Rect(lt.x, lt.y, rb.x - lt.x, rb.y - lt.y);

        cv::Mat croped_image;
        try
          {
            croped_image = image(roi);
          }
        catch (const cv::Exception& e)
          {
            std::cerr << "failed to crop image: " << e.what() << std::endl;
          }

        autoware_traffic_light_msgs::LampStateArray lamp_state_array;

        cv::Rect projected_roi(stamped_roi_msg->roi_array.at(i).x_offset,
                               stamped_roi_msg->roi_array.at(i).y_offset,
                               stamped_roi_msg->roi_array.at(i).width,
                               stamped_roi_msg->roi_array.at(i).height);


        if ( !classify(croped_image, lamp_state_array, color_debug_image, arrow_debug_image, projected_roi) )
          {
            ROS_ERROR("failed classify image, abort callback");
            return;
          }

        std::copy(lamp_state_array.states.begin(),
                  lamp_state_array.states.end(),
                  std::back_inserter(trafficlight_state.states));
        trafficlight_state.header = stamped_roi_msg->header;

        trafficlight_state_array.states.push_back(trafficlight_state);

      }

    trafficlight_state_array.header = stamped_roi_msg->header;
    trafficlight_state_array_pub_.publish(trafficlight_state_array);

    image_pub_.publish(cv_bridge::CvImage
                       (stamped_roi_msg->header,
                        sensor_msgs::image_encodings::BGR8,
                        color_debug_image).toImageMsg());

    arrow_debug_pub_.publish(cv_bridge::CvImage
                             (stamped_roi_msg->header,
                              sensor_msgs::image_encodings::BGR8,
                              arrow_debug_image).toImageMsg());

  }


  void TrafficLightClassifierNode::run()
  {
    ROS_INFO("run");
    ros::NodeHandle pnh("~");
    pnh.getParam("approximate_sync", is_approximate_sync_);

    color_classifier_ = std::make_shared<ColorClassifier>();
    arrow_classifier_ = std::make_shared<ArrowClassifier>();
    utils_ = std::make_shared<Utils>();

    if( !color_classifier_->get_params(pnh) )
      {
        ROS_ERROR("failed get color calssfier params");
        return;
      }

    if( !arrow_classifier_->get_params(pnh) )
      {
        ROS_ERROR("failed get arrow calssfier params");
        return;
      }

    image_pub_ =
      pnh.advertise<sensor_msgs::Image>("output_image", 1);

    arrow_debug_pub_ =
      pnh.advertise<sensor_msgs::Image>("output_arrow_debug", 1);

    trafficlight_state_array_pub_ =
      pnh.advertise<autoware_traffic_light_msgs::TrafficLightStateArray>("output_light_states", 1);

    image_sub_.subscribe(pnh, "input_image", 1);
    stamped_roi_sub_.subscribe(pnh, "input_stamped_roi", 1);

    if (is_approximate_sync_)
      {
        approximate_sync_ =
          boost::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy> >(100);
        approximate_sync_->connectInput(image_sub_, stamped_roi_sub_);
        approximate_sync_->registerCallback
          (boost::bind(&TrafficLightClassifierNode::callback, this, _1, _2));
      }
    else
      {
        sync_  =
          boost::make_shared<message_filters::Synchronizer<SyncPolicy> >(100);
        sync_->connectInput(image_sub_, stamped_roi_sub_);
        sync_->registerCallback
          (boost::bind(&TrafficLightClassifierNode::callback, this, _1, _2));
      }

    ros::spin();
  }

}

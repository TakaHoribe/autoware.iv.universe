#include "new_trafficlight_recognizer/kcf_tracker.h"

namespace new_trafficlight_recognizer
{

  /// KcfTracker

  KcfTracker::KcfTracker(const cv::Rect& init_box,
                         const std::list<cv::Mat>& interpolation_images,
                         const bool init_box_stamp_changed)
  {
    init_box_ = cv::Rect(init_box.x - offset_.x,
                         init_box.y - offset_.y,
                         init_box.width + offset_.x * 2,
                         init_box.height + offset_.y * 2);
    initialize_ = init_box_stamp_changed;
    std::copy(interpolation_images.begin(),
              interpolation_images.end(),
              std::back_inserter(interpolation_images_));
  }

  bool KcfTracker::update_parameters(const cv::Rect& init_box,
                                     const std::list<cv::Mat>& interpolation_images,
                                     const bool init_box_stamp_changed)
  {
    init_box_ = cv::Rect(init_box.x - offset_.x,
                         init_box.y - offset_.y,
                         init_box.width + offset_.x * 2,
                         init_box.height + offset_.y * 2);
    initialize_ = init_box_stamp_changed;
    std::copy(interpolation_images.begin(),
              interpolation_images.end(),
              std::back_inserter(interpolation_images_));

    return true;

  }

  bool KcfTracker::update_tracker(std::list<cv::Mat>& interpolation_images,
                                  cv::Rect& output_box)
  {

    try
      {
        int num_interpolation = interpolation_images.size();
        for (int i=0; i<num_interpolation; ++i)
          {
            cv::Mat image = interpolation_images.front();

            track(image);

            interpolation_images.pop_front();
          }

        BBox_c bb = getBBox();
        output_box = cv::Rect
          (bb.cx - bb.w * 0.5, bb.cy - bb.h * 0.5, bb.w, bb.h);
      }

    catch (const std::out_of_range& e)
      {
        std::cerr << "failed to update tracker" << std::endl;
        std::cerr << "Out of Range error: " << e.what() << std::endl;
        return false;
      }

    return true;
  }

  bool KcfTracker::is_box_in_image(const cv::Mat& image,
                                   const cv::Rect& box)
  {
    if (box.x < 0 || box.y < 0 ||
        box.x + box.width > image.cols ||
        box.y + box.height > image.rows)
      {
        std::cerr << "box size is over input image" << std::endl;
        return false;
      }
  }

  bool KcfTracker::run(cv::Rect& output_box)
  {

    if ( interpolation_images_.empty() )
      {
        std::cerr << "interpolation_images is empty" << std::endl;
        return false;
      }

    if (initialize_)
      {
        // init
        try
          {
            cv::Mat init_image = interpolation_images_.front();
            interpolation_images_.pop_front();

            if ( !is_box_in_image(init_image, init_box_) )
              {
                return false;
              }

            init(init_image, init_box_);

          }
        catch (const std::out_of_range& e)
          {
            std::cerr << "Out of Range error: " << e.what() << std::endl;
            return false;
          }


        // update if interpolation_images_ size > 0
        if ( !update_tracker(interpolation_images_, output_box) )
          {
            return false;
          }

      }
    else
      {

        // update
        if ( !update_tracker(interpolation_images_, output_box) )
          {
            return false;
          }
      }

    return true;

  }


  /// MultiKcfTracker

  MultiKcfTracker::MultiKcfTracker(int buffer_size,
                                   int interpolation_frequency)
  {
    buffer_size_ = buffer_size;
    interpolation_frequency_ = interpolation_frequency;
  }

  bool MultiKcfTracker::push_to_tracker_map(std::map<int, KcfTrackerPtr>& tracker_map,
                                            const int signal,
                                            const cv::Rect& init_box,
                                            const std::list<cv::Mat>& interpolation_images,
                                            const bool init_box_stamp_changed)
  {
    // push tracker
    try
      {
        if ( tracker_map.find(signal) == tracker_map.end() )
          {
            KcfTrackerPtr tracker = std::make_shared<KcfTracker>
              (init_box, interpolation_images, init_box_stamp_changed);
            tracker_map.emplace(signal, tracker);
          }
        else
          {
            auto kcf_tracker = tracker_map.at(signal);
            kcf_tracker->update_parameters
              (init_box, interpolation_images, init_box_stamp_changed);
          }
      }
    catch (const std::out_of_range& e)
      {
        std::cerr << "Out of Range error: " << e.what() << std::endl;
        return false;
      }

    return true;
  }


  bool MultiKcfTracker::pop_from_tracker_map(std::map<int, KcfTrackerPtr>& tracker_map,
                                             const std::vector<int>& signals)
  {
    try
      {
        for (auto it = tracker_map.begin(); it != tracker_map.end(); ++it)
          {
            // key not in signals
            if ( signals.end() == std::find(signals.begin(), signals.end(), it->first) )
              {
                tracker_map.erase(it->first);
              }
          }
      }
    catch (...)
      {
        std::cerr << "exception occur" << std::endl;
        return false;
      }

    return true;
  }

  bool MultiKcfTracker::create_buffer(const cv::Mat& image,
                                      const double image_stamp)
  {
    try
      {
        if (image_buffer_.size() >= buffer_size_)
          {
            image_buffer_.erase(image_buffer_.begin());
            image_stamp_buffer_.erase(image_stamp_buffer_.begin());
          }
        image_buffer_.push_back(image);
        image_stamp_buffer_.push_back(image_stamp);
      }
    catch (...)
      {
        std::cerr << "failed to create buffer" << std::endl;
        return false;
      }

    return true;
  }

  bool MultiKcfTracker::get_init_box_stamped_index(const std::vector<double>& image_stamp_buffer,
                                                   int& stamped_index,
                                                   double init_box_stamp)
  {
    stamped_index = 0;
    bool found_stamped_index = false;

    if (image_stamp_buffer.empty())
      {
        std::cerr << "image_stamp_buffer is empty" << std::endl;
        return false;
      }

    if (init_box_stamp > image_stamp_buffer.back())
      {
        std::cerr << "init_box_stamp should be less than latest image_stamp" << std::endl;
        return false;
      }


    for (int i=image_stamp_buffer.size() - 1; i>=0; i--)
      {

        if (image_stamp_buffer.at(i) - init_box_stamp <= 0)
          {

            found_stamped_index = true;
            if (std::abs(image_stamp_buffer.at(i+1) - init_box_stamp) <
                std::abs(image_stamp_buffer.at(i) - init_box_stamp))
              {
                stamped_index = i+1;
              }
            else
              {
                stamped_index = i;
              }
            break;
          }
      }

    return found_stamped_index;
  }

  bool MultiKcfTracker::get_interpolation_images(const bool init_box_stamp_changed,
                                                 const double init_box_stamp,
                                                 const std::vector<cv::Mat>& image_buffer,
                                                 const std::vector<double>& image_stamp_buffer,
                                                 std::list<cv::Mat>& interpolation_images)
  {
    if ( image_buffer.empty() || image_stamp_buffer.empty())
      {
        std::cerr << "buffe is empty" << std::endl;
        return false;
      }

    if ( image_buffer.size() != image_stamp_buffer.size() )
      {
        std::cerr << "image_buffer and image_stamp_buffer size should be same" << std::endl;
        return false;
      }

    if ( init_box_stamp_changed )
      {
        int stamped_index = 0;

        if ( !get_init_box_stamped_index(image_stamp_buffer, stamped_index, init_box_stamp))
          {
            std::cerr << "failed to get init_box_stamped_index" << std::endl;
            return false;
          }

        for (int i=stamped_index; i<image_buffer.size(); i+=interpolation_frequency_)
          {
            interpolation_images.push_back(image_buffer.at(i));
          }
      }
    else
      {
        interpolation_images.push_back(image_buffer.back());
      }

    return true;
  }


  bool MultiKcfTracker::run(const std::vector<int>& signals,
                            const std::vector<int>& init_boxes_signals,
                            const cv::Mat& original_image,
                            const std::vector<cv::Rect>& init_boxes,
                            const double image_stamp,
                            const double init_box_stamp,
                            std::vector<cv::Rect>& output_boxes,
                            std::vector<int>& output_signals,
                            cv::Mat& debug_image)
  {

    // images buffer for interpolation
    if (! create_buffer(original_image, image_stamp) )
      {
        prev_init_box_stamp_ = init_box_stamp;
        return false;
      }

    if ( init_box_stamp != 0 )
      {
        // create interpolation images caused by detection latency
        std::list<cv::Mat> interpolation_images;
        bool init_box_stamp_changed = init_box_stamp != prev_init_box_stamp_;

        if ( !get_interpolation_images
             (init_box_stamp_changed, init_box_stamp,
              image_buffer_, image_stamp_buffer_, interpolation_images))
          {
            prev_init_box_stamp_ = init_box_stamp;
            return false;
          }

        // remove unnecessary tracker map
        // do not track object that could not detect by tlr_detector
        if ( !pop_from_tracker_map(tracker_map_, init_boxes_signals) )
          {
            prev_init_box_stamp_ = init_box_stamp;
            return false;
          }

        debug_image = interpolation_images.back().clone();

        for (int i=0; i<init_boxes_signals.size(); ++i)
          {
            auto signal = init_boxes_signals.at(i);
            auto init_box = init_boxes.at(i);

            if ( !push_to_tracker_map
                 (tracker_map_,
                  signal,
                  init_box,
                  interpolation_images,
                  init_box_stamp_changed) )
              {
                prev_init_box_stamp_ = init_box_stamp;
                return false;
              }

            cv::Rect output_box;
            auto kcf_tracker = tracker_map_.at(signal);

            if ( !kcf_tracker->run(output_box) )
              {
                return false;
              }

            cv::rectangle(debug_image, output_box, CV_RGB(0,0,255), 2);

            output_boxes.push_back(output_box);
            output_signals.push_back(signal);
          }

      }
    else
      {
      }

    prev_init_box_stamp_ = init_box_stamp;

    return true;

  }

} // namespace new_trafficlight_recognizer

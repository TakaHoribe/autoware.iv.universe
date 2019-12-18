/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <iostream>
#include <gtest/gtest.h>
#include "new_trafficlight_recognizer/kcf_tracker_node.h"

class KcfTrackerNodeTestSuite : public ::testing::Test
{
protected:

  typedef std::shared_ptr<new_trafficlight_recognizer::MultiKcfTracker> MultiKcfTrackerPtr;

  typedef std::shared_ptr<new_trafficlight_recognizer::KcfTracker> KcfTrackerPtr;

  MultiKcfTrackerPtr multi_kcf_tracker_;

  KcfTrackerPtr kcf_tracker_;

  int buffer_size_ = 10;

  int interpolation_frequency_ = 1;


  virtual void SetUp()
  {
    generateTestData();
  }

  virtual void generateTestData()
  {
  }

};


TEST_F(KcfTrackerNodeTestSuite, createBuffer)
{
  std::cerr << "\n" << __func__ << ":createBuffer" << std::endl;

  multi_kcf_tracker_ = std::make_shared<new_trafficlight_recognizer::MultiKcfTracker>
    (buffer_size_, interpolation_frequency_);

  for (int i=0; i<30; ++i)
    {
      cv::Mat original_image = cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0,0,255));
      int image_stamp = i;
      multi_kcf_tracker_->create_buffer(original_image, image_stamp);
    }

  ASSERT_EQ(multi_kcf_tracker_->image_buffer_.size(),
            multi_kcf_tracker_->image_stamp_buffer_.size())
    << "image_buffer_ and image_stamp_buffer_ size should be same. ";

  ASSERT_EQ(multi_kcf_tracker_->image_buffer_.size(), buffer_size_)
    << "image_buffer_ and buffer_size should be same. ";

}

TEST_F(KcfTrackerNodeTestSuite, getInitBoxStampedIndex)
{
  std::cerr << "\n" << __func__ << ":getInitBoxStampedIndex" << std::endl;

  multi_kcf_tracker_ = std::make_shared<new_trafficlight_recognizer::MultiKcfTracker>
    (buffer_size_, interpolation_frequency_);

  std::vector<double> image_stamp_buffer{0,1,2,3,4,5};
  int stamped_index = 0;
  double init_box_stamp = 4;
  bool is_succeeded = multi_kcf_tracker_->get_init_box_stamped_index
    (image_stamp_buffer, stamped_index, init_box_stamp);

  ASSERT_EQ(true, is_succeeded) << "failed to get init_box_stamped_index";

  ASSERT_EQ(4, stamped_index) << "stamped_index is invalid value";


  image_stamp_buffer = {100,101,102};
  stamped_index = 0;
  init_box_stamp = 100;
  is_succeeded = multi_kcf_tracker_->get_init_box_stamped_index
    (image_stamp_buffer, stamped_index, init_box_stamp);

  ASSERT_EQ(true, is_succeeded) << "failed to get init_box_stamped_index";

  ASSERT_EQ(0, stamped_index) << "stamped_index is invalid value";


  // test for invalid value
  double init_box_stamp_invalid = 1000;
  is_succeeded = multi_kcf_tracker_->get_init_box_stamped_index
    (image_stamp_buffer, stamped_index, init_box_stamp_invalid);

  ASSERT_EQ(false, is_succeeded) << "init_box_stamp value should not be found in image_stamp_buffer";

  std::vector<double> image_stamp_buffer_invalid{};
  is_succeeded = multi_kcf_tracker_->get_init_box_stamped_index
    (image_stamp_buffer_invalid, stamped_index, init_box_stamp);

  ASSERT_EQ(false, is_succeeded) << "image_stamp_buffer should have at least a stamp";

}

TEST_F(KcfTrackerNodeTestSuite, getInterpolationImages)
{
  std::cerr << "\n" << __func__ << ":getInterpolationImages" << std::endl;

  multi_kcf_tracker_ = std::make_shared<new_trafficlight_recognizer::MultiKcfTracker>
    (buffer_size_, interpolation_frequency_);

  bool init_box_stamp_changed = true;
  double init_box_stamp = 5;
  std::vector<cv::Mat> image_buffer(buffer_size_);
  std::vector<double> image_stamp_buffer(buffer_size_);

  for (int i=0; i<buffer_size_; ++i)
    {
      cv::Mat original_image = cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0,0,255));
      image_buffer.at(i) = original_image;
      image_stamp_buffer.at(i) = i;
    }

  std::list<cv::Mat> interpolation_images;
  bool is_succeeded = multi_kcf_tracker_->get_interpolation_images
    (init_box_stamp_changed,
     init_box_stamp,
     image_buffer,
     image_stamp_buffer,
     interpolation_images);

  ASSERT_EQ(true, is_succeeded) << "failed to get interpolation images";

  ASSERT_EQ(buffer_size_ - init_box_stamp, interpolation_images.size())
    << "interpolation images size should be buffer_size - init_box_stamp size";

  std::vector<double> image_stamp_buffer_invalid{};
  is_succeeded = multi_kcf_tracker_->get_interpolation_images
    (init_box_stamp_changed,
     init_box_stamp,
     image_buffer,
     image_stamp_buffer_invalid,
     interpolation_images);

  ASSERT_EQ(false, is_succeeded) << "image_stamp_buffer should have at least a stamp";

}

TEST_F(KcfTrackerNodeTestSuite, pushAndPopTrackerMap)
{
  std::cerr << "\n" << __func__ << ":pushAndPopTrackerMap" << std::endl;

  multi_kcf_tracker_ = std::make_shared<new_trafficlight_recognizer::MultiKcfTracker>
    (buffer_size_, interpolation_frequency_);

  std::map<int, KcfTrackerPtr> tracker_map;

  std::list<cv::Mat> interpolation_images;
  for (int i=0; i<buffer_size_; ++i)
    {
      interpolation_images.push_back
        (cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0,0,255)));
    }

  std::vector<int> signals1{1,2};
  for (int i=0; i<signals1.size(); ++i)
    {
      int signal = signals1.at(i);
      cv::Rect init_box(20*i+5, 20*i+5, 10, 10);
      bool init_box_stamp_changed = true;

      bool is_succeeded = multi_kcf_tracker_->push_to_tracker_map
        (tracker_map,
         signal,
         init_box,
         interpolation_images,
         init_box_stamp_changed);

      ASSERT_EQ(true, is_succeeded) << "failed to push to tracker_map";

    }

  // expect tracker_map keys {1,2}
  ASSERT_EQ(tracker_map.size(), 2) << "expect tracker_map keys {1,2}";
  ASSERT_EQ(tracker_map.cbegin()->first, 1) << "expect tracker_map keys {1,2}";


  std::vector<int> signals2{2,3,4,5};
  // pop
  bool is_succeeded = multi_kcf_tracker_->pop_from_tracker_map(tracker_map, signals2);
  ASSERT_EQ(true, is_succeeded) << "failed to pop from tracker_map";

  // expect tracker_map keys {2}
  ASSERT_EQ(tracker_map.size(), 1) << "expect tracker_map keys {2}" ;
  ASSERT_EQ(tracker_map.cbegin()->first, 2) << "expect tracker_map keys {2}";


  // push
  for (int i=0; i<signals2.size(); ++i)
    {
      int signal = signals2.at(i);
      cv::Rect init_box(20*i+5, 20*i+5, 10, 10);
      bool init_box_stamp_changed = true;

      bool is_succeeded = multi_kcf_tracker_->push_to_tracker_map
        (tracker_map,
         signal,
         init_box,
         interpolation_images,
         init_box_stamp_changed);

      ASSERT_EQ(true, is_succeeded) << "failed to push to tracker_map";

    }

  // expect tracker_map keys {2,3,4,5}
  ASSERT_EQ(tracker_map.size(), 4) << "expect tracker_map keys {2,3,4,5}" ;
  ASSERT_EQ(tracker_map.cbegin()->first, 2) << "expect tracker_map keys {2,3,4,5}";

}


TEST_F(KcfTrackerNodeTestSuite, runMultiKcfTracking)
{
  std::cerr << "\n" << __func__ << ":runMultiKcfTracking" << std::endl;

  multi_kcf_tracker_ = std::make_shared<new_trafficlight_recognizer::MultiKcfTracker>
    (buffer_size_, interpolation_frequency_);

  std::vector< std::vector<int> > sequential_signals
  {{1,2,3}, {1,2,3}, {1,3,4}, {1,3,4}, {1,3,4}};
  std::vector< std::vector<int> > sequential_detected_boxes_signals
  {{1,2,3}, {1,2,3}, {1,3,4}, {1,3,4}, {1,3,4}};

  std::vector<std::vector <cv::Rect> > sequential_init_boxes
  {{cv::Rect(100, 100, 50, 50), cv::Rect(200, 200, 50, 50), cv::Rect(300, 300, 50, 50)},
   {cv::Rect(100, 100, 50, 50), cv::Rect(200, 200, 50, 50), cv::Rect(300, 300, 50, 50)},
   {cv::Rect(100, 100, 50, 50), cv::Rect(330, 330, 50, 50), cv::Rect(400, 400, 50, 50)},
   {cv::Rect(100, 100, 50, 50), cv::Rect(330, 330, 50, 50), cv::Rect(400, 400, 50, 50)},
   {cv::Rect(100, 100, 50, 50), cv::Rect(330, 330, 50, 50), cv::Rect(400, 400, 50, 50)}};

  std::vector<cv::Mat> sequential_original_image
  {cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0,0,255)),
   cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0,0,255)),
   cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0,0,255)),
   cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0,0,255)),
   cv::Mat(cv::Size(640, 480), CV_8UC3, cv::Scalar(0,0,255))};

  std::vector<double> sequential_image_stamp
  {100, 101, 102, 103, 104};

  std::vector<double> sequential_init_box_stamp
  {0, 0, 100, 100, 100};

  std::vector<int> expect_output_box_sizes
  {0, 0, 3, 3, 3};

  for (int i=0; i<sequential_signals.size(); ++i)
    {
      std::vector<int> signals = sequential_signals.at(i);
      std::vector<int> detected_boxes_signals = sequential_detected_boxes_signals.at(i);

      cv::Mat original_image = sequential_original_image.at(i);
      std::vector<cv::Rect> init_boxes = sequential_init_boxes.at(i);
      double image_stamp = sequential_image_stamp.at(i);
      double init_box_stamp = sequential_init_box_stamp.at(i);

      std::vector<cv::Rect> output_boxes;
      std::vector<int> output_signals;
      cv::Mat debug_image;

      bool succeeded = multi_kcf_tracker_->run
        (signals,
         detected_boxes_signals,
         original_image,
         init_boxes,
         image_stamp,
         init_box_stamp,
         output_boxes,
         output_signals,
         debug_image);

      ASSERT_EQ(true, succeeded) << "failed to run multi_kcf_tracker";

      double expect_size = expect_output_box_sizes.at(i);
      ASSERT_EQ(output_boxes.size(), expect_size) << "expect size is {0, 0, 3, 3, 3}" ;
      ASSERT_EQ(output_signals.size(), expect_size) << "expect size is {0, 0, 3, 3, 3}" ;
    }
}


int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "KcfTrackerNodeTestSuite");

  return RUN_ALL_TESTS();
}

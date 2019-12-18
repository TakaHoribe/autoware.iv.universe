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
#include "new_trafficlight_recognizer/feat_proj_bbox.h"

class FeatProjBBoxTestSuite : public ::testing::Test
{
protected:

  new_trafficlight_recognizer::FeatProjBBox extractor_;

  int signals_size;
  cv::Size image_size;
  cv::Size zero_size;
  std::vector<autoware_msgs::ExtractedPosition> signals;

  virtual void SetUp()
  {
    signals_size = 5;
    generateTestData();
  }

  virtual void generateTestData()
  {
    image_size = cv::Size(100, 100);
    zero_size = cv::Size(0,0);

    for (int i=0; i<signals_size; ++i)
      {
        autoware_msgs::ExtractedPosition test_extracted_position;
        test_extracted_position.signalId = 0;
        test_extracted_position.u = 0;
        test_extracted_position.v = 0;
        test_extracted_position.z = 0;
        signals.push_back(test_extracted_position);
      }
  }

};


TEST_F(FeatProjBBoxTestSuite, extractProjectedArea)
{
  autoware_traffic_light_msgs::StampedRoi projected_rois;
  bool is_succeeded = extractor_.extract(image_size, signals, projected_rois);
  ASSERT_EQ(true, is_succeeded) << "failed to extract projected rois";

  // test for invalid value
  is_succeeded = extractor_.extract(zero_size, signals, projected_rois);
  ASSERT_EQ(false, is_succeeded) << "miss-handle zero size image";

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "FeatProjBBoxTestSuite");

  return RUN_ALL_TESTS();
}

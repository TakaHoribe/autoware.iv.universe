/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
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

#include <autoware_planner_msgs/Trajectory.h>
#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <iostream>
#include "libplanning_utils/planning_utils.h"
#include "libplanning_utils/pure_pursuit.h"

using namespace planning_utils;

constexpr double ERROR = 1e-6;

class TestSuite : public ::testing::Test {
 public:
  TestSuite() {}
  ~TestSuite() {}
};

TEST_F(TestSuite, PurePursuit_is_requirements_safisfied) {
  PurePursuit pp;
  ASSERT_EQ(false, pp.isRequirementsSatisfied());

  geometry_msgs::PoseStampedConstPtr pose_ptr(new geometry_msgs::PoseStamped());
  autoware_planner_msgs::TrajectoryConstPtr wps_ptr(new autoware_planner_msgs::Trajectory());
  pp.setCurrentPose(pose_ptr->pose);
  pp.setWaypoints(extractPoses(*wps_ptr));
  ASSERT_EQ(true, pp.isRequirementsSatisfied());
}

TEST_F(TestSuite, PurePursuit_run) {
  PurePursuit pp;
  auto res = pp.run();
  ASSERT_EQ(false, res.first);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
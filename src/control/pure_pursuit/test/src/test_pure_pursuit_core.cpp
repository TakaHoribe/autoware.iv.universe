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

#include <gtest/gtest.h>
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cmath>
#include <iostream>

#include "pure_pursuit/pure_pursuit_core.h"

using namespace waypoint_follower;

constexpr double ERROR = 1e-6;

class TestSuite : public ::testing::Test {
 public:
  TestSuite() {}
  ~TestSuite() {}
};

TEST_F(TestSuite, Test) {}

TEST_F(TestSuite, computeLookaheadDistance) {
  PurePursuitDynamicConfig ppdconf;
  double curr_linear_vel = 0.0;
  double cmd_vel = 0.0;
  double res = 0.0;

  // param config = 0(dialog)
  ppdconf.param_flag_ = enumToInteger(Mode::dialog);
  res = computeLookaheadDistance(ppdconf, curr_linear_vel, cmd_vel);
  ASSERT_NEAR(ppdconf.const_lookahead_distance_, res, ERROR);

  // param config = 1(waypoint)
  ppdconf.param_flag_ = enumToInteger(Mode::waypoint);
  res = computeLookaheadDistance(ppdconf, curr_linear_vel, cmd_vel);
  ASSERT_NEAR(ppdconf.minimum_lookahead_distance_, res, ERROR);

  cmd_vel = -1.0;
  res = computeLookaheadDistance(ppdconf, curr_linear_vel, cmd_vel);
  ASSERT_NEAR(ppdconf.reverse_minld_, res, ERROR);

  cmd_vel = 1.0;
  curr_linear_vel = 3.0;
  res = computeLookaheadDistance(ppdconf, curr_linear_vel, cmd_vel);
  ASSERT_NEAR(curr_linear_vel * ppdconf.lookahead_distance_ratio_, res, ERROR);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
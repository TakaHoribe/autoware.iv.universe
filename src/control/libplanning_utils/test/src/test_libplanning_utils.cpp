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

#include "libplanning_utils/planning_utils.h"

using namespace planning_utils;

class TestSuite : public ::testing::Test {
 public:
  TestSuite() {}
  ~TestSuite() {}
};

TEST_F(TestSuite, calcCurvature) {
  geometry_msgs::Point target;
  geometry_msgs::Pose curr_pose;

  target.x = 10.0;
  target.y = 0.0;
  ASSERT_NEAR(0.0, calcCurvature(target, curr_pose), ERROR);

  target.x = 10.0;
  target.y = 10.0;
  ASSERT_NEAR(0.1, calcCurvature(target, curr_pose), ERROR);

  target.x = 0.0;
  target.y = 10.0;
  ASSERT_NEAR(0.2, calcCurvature(target, curr_pose), ERROR);
}

TEST_F(TestSuite, calcDistSquared2D) {
  geometry_msgs::Point p;
  geometry_msgs::Point q;

  // p.x = 0.0, p.y = 0.0, q.x = 0.0, q.y = 0.0, length2=0.0m
  ASSERT_NEAR(0.000000, calcDistSquared2D(p, q), ERROR);

  // p.x = 0.0, p.y = 0.0, p.z = 10.0, q.x = 0.0, q.y = 0.0, length2=0.0m, z is invalid value
  p.z = 10.0;
  ASSERT_NEAR(0.000000, calcDistSquared2D(p, q), ERROR);

  // p.x = 1.0, p.y = 0.0, q.x = 0.0, q.y = 0.0, length2=0.0m
  p.x = 1.0;
  p.z = 0.0;
  ASSERT_NEAR(1.000000, calcDistSquared2D(p, q), ERROR);

  // p.x = 2.0, p.y = 6.0, q.x = 6.0, q.y = 3.0, length2=25.0m
  p.x = 2.0;
  p.y = 6.0;
  q.x = 6.0;
  q.y = 3.0;
  ASSERT_NEAR(25.000000, calcDistSquared2D(p, q), ERROR);
}

TEST_F(TestSuite, calcLateralError) {
  geometry_msgs::Point line_s;
  geometry_msgs::Point line_e;
  geometry_msgs::Point point;

  // target point is on right side of the line
  line_s.x = 2.0;
  line_s.y = 4.0;

  line_e.x = 7.0;
  line_e.y = 3.0;

  point.x = 6.0;
  point.y = -3.0;
  ASSERT_NEAR(-6.079600, calcLateralError2D(line_s, line_e, point), ERROR);

  // target point is on left side of the line
  point.x = 4.0;
  point.y = 8.0;
  ASSERT_NEAR(4.314555, calcLateralError2D(line_s, line_e, point), ERROR);

  // the length of line is zero
  ASSERT_NEAR(0.0, calcLateralError2D(line_s, line_s, point), ERROR);
}

TEST_F(TestSuite, calcRadius) {
  geometry_msgs::Point target;
  geometry_msgs::Pose curr_pose;

  ASSERT_NEAR(1e9, calcRadius(target, curr_pose), ERROR);

  target.x = 10.0;
  target.y = 0.0;
  ASSERT_NEAR(1e9, calcRadius(target, curr_pose), ERROR);

  target.x = 10.0;
  target.y = 10.0;
  ASSERT_NEAR(10.0, calcRadius(target, curr_pose), ERROR);

  target.x = 0.0;
  target.y = 10.0;
  ASSERT_NEAR(5.0, calcRadius(target, curr_pose), ERROR);
}

TEST_F(TestSuite, convertCurvatureToSteeringAngle) {
  // wheel base = 4.0m, kappa = 0.002m (R = 500m)
  ASSERT_NEAR(0.008000, convertCurvatureToSteeringAngle(4.0, 0.002), ERROR);

  // wheel base = 4.0m, kappa = -0.000125m (R = -8000m)
  ASSERT_NEAR(-0.000500, convertCurvatureToSteeringAngle(4.0, -0.000125), ERROR);
}
/*
TEST_F(TestSuite, extractPoses_Lane)
{
  autoware_msgs::Lane test;
  test.waypoints.emplace_back();
  test.waypoints.emplace_back();
  test.waypoints.emplace_back();
  test.waypoints.at(0).pose.pose.position.x = 5.0;
  test.waypoints.at(1).pose.pose.position.x = 100.0;
  test.waypoints.at(2).pose.pose.position.x = -999.0;

  std::vector<geometry_msgs::Pose> poses = extractPoses(test);
  ASSERT_NEAR(5.0, test.waypoints.at(0).pose.pose.position.x, ERROR);
  ASSERT_NEAR(100.0, test.waypoints.at(1).pose.pose.position.x, ERROR);
  ASSERT_NEAR(-999.0, test.waypoints.at(2).pose.pose.position.x, ERROR);
}

TEST_F(TestSuite, extractPoses_Waypoint)
{
  std::vector<autoware_msgs::Waypoint> test;
  test.emplace_back();
  test.emplace_back();
  test.emplace_back();
  test.at(0).pose.pose.position.x = 5.0;
  test.at(1).pose.pose.position.x = 100.0;
  test.at(2).pose.pose.position.x = -999.0;

  std::vector<geometry_msgs::Pose> poses = extractPoses(test);
  ASSERT_NEAR(5.0, test.at(0).pose.pose.position.x, ERROR);
  ASSERT_NEAR(100.0, test.at(1).pose.pose.position.x, ERROR);
  ASSERT_NEAR(-999.0, test.at(2).pose.pose.position.x, ERROR);
}
*/
TEST_F(TestSuite, extractPoses_template_Motions) {
  std::vector<autoware_planner_msgs::Motion> test;
  test.emplace_back();
  test.emplace_back();
  test.emplace_back();
  test.at(0).pose.position.x = 5.0;
  test.at(1).pose.position.x = 100.0;
  test.at(2).pose.position.x = -999.0;

  std::vector<geometry_msgs::Pose> poses = extractPoses(test);
  ASSERT_NEAR(5.0, test.at(0).pose.position.x, ERROR);
  ASSERT_NEAR(100.0, test.at(1).pose.position.x, ERROR);
  ASSERT_NEAR(-999.0, test.at(2).pose.position.x, ERROR);
}

TEST_F(TestSuite, findClosestIdxWithDistAngThr) {
  std::vector<geometry_msgs::Pose> curr_ps;
  geometry_msgs::Pose curr_pose;

  std::pair<bool, int32_t> ans;

  ans = findClosestIdxWithDistAngThr(curr_ps, curr_pose);
  ASSERT_EQ(false, ans.first);
  ASSERT_EQ(-1, ans.second);

  tf2::Quaternion tf2_q;
  tf2_q.setRPY(0, 0, 0);
  tf2_q.normalize();
  for (uint32_t i = 0; i < 10; i++) {
    geometry_msgs::Pose tmp_pose;
    tmp_pose.position.x = (double)i * 1.0;
    tmp_pose.orientation = tf2::toMsg(tf2_q);
    curr_ps.push_back(tmp_pose);
  }

  curr_pose.position.x = 0.0;
  curr_pose.orientation = tf2::toMsg(tf2_q);
  ans = findClosestIdxWithDistAngThr(curr_ps, curr_pose);
  ASSERT_EQ(true, ans.first);
  ASSERT_EQ(0, ans.second);

  tf2_q.setRPY(0, 0, 0.175);  // yaw = 10deg
  tf2_q.normalize();
  curr_pose.position.x = 3.0;
  curr_pose.orientation = tf2::toMsg(tf2_q);
  ans = findClosestIdxWithDistAngThr(curr_ps, curr_pose);
  ASSERT_EQ(true, ans.first);
  ASSERT_EQ(3, ans.second);
}

TEST_F(TestSuite, kmph2mps) {
  double kmph = 5.0;
  ASSERT_DOUBLE_EQ(kmph / 3.6, kmph2mps(kmph));

  kmph = 9.0;
  ASSERT_DOUBLE_EQ(kmph / 3.6, kmph2mps(kmph));
}

TEST_F(TestSuite, isDirectionForward) {
  std::vector<geometry_msgs::Pose> poses;
  poses.reserve(2);
  geometry_msgs::Pose p0;
  geometry_msgs::Pose p1;

  // p0 and 01 is same pose
  poses.push_back(p0);
  poses.push_back(p1);
  ASSERT_EQ(false, isDirectionForward(poses));

  // p1.position.x = 3.0 p0 is origin position
  poses.clear();
  poses.push_back(p0);
  p1.position.x = 3.0;
  poses.push_back(p1);
  ASSERT_EQ(true, isDirectionForward(poses));

  // p1.position.x = -3.0 p0 is origin position
  poses.clear();
  poses.push_back(p0);
  p1.position.x = -3.0;
  poses.push_back(p1);
  ASSERT_EQ(false, isDirectionForward(poses));
}

TEST_F(TestSuite, isInPolygon_template_geometry_msgs_Point) {
  std::vector<geometry_msgs::Point> polygon;
  geometry_msgs::Point point;

  // 0 point polygon
  ASSERT_EQ(false, isInPolygon<geometry_msgs::Point>(polygon, point));

  // 1 point polygon
  polygon.reserve(1);
  polygon.emplace_back();
  ASSERT_EQ(false, isInPolygon<geometry_msgs::Point>(polygon, point));

  // 2 point polygon
  polygon.clear();
  polygon.shrink_to_fit();
  polygon.reserve(2);
  polygon.emplace_back();
  polygon.emplace_back();
  ASSERT_EQ(false, isInPolygon<geometry_msgs::Point>(polygon, point));

  // 3 point polygon
  polygon.clear();
  polygon.shrink_to_fit();
  polygon.reserve(3);
  polygon.push_back(Eigen::toMsg(Eigen::Vector3d(0.001, 0.0, 0.0)));
  polygon.push_back(Eigen::toMsg(Eigen::Vector3d(3.0, 2.0, 0.0)));
  polygon.push_back(Eigen::toMsg(Eigen::Vector3d(0.001, 4.0, 0.0)));
  // point is on the edge of 3 point polygon
  point.x = 0.001;
  point.y = 2.0;
  ASSERT_EQ(false, isInPolygon<geometry_msgs::Point>(polygon, point));

  // point is inside of 3 point polygon
  point.x = 0.001001;
  point.y = 2.0;
  ASSERT_EQ(true, isInPolygon<geometry_msgs::Point>(polygon, point));

  // point is inside of 3 point polygon
  point.x = 2.0;
  point.y = 2.0;
  ASSERT_EQ(true, isInPolygon<geometry_msgs::Point>(polygon, point));

  // 4 point polygon
  polygon.clear();
  polygon.shrink_to_fit();
  polygon.reserve(4);
  polygon.push_back(Eigen::toMsg(Eigen::Vector3d(0.0, 0.0, 0.0)));
  polygon.push_back(Eigen::toMsg(Eigen::Vector3d(3.0, 0.0, 0.0)));
  polygon.push_back(Eigen::toMsg(Eigen::Vector3d(3.0, 4.0, 0.0)));
  polygon.push_back(Eigen::toMsg(Eigen::Vector3d(0.0, 4.0, 0.0)));
  // point is on the edge of 4 point polygon
  point.x = 0.0;
  point.y = 2.0;
  ASSERT_EQ(false, isInPolygon<geometry_msgs::Point>(polygon, point));

  // point is on the edge of 4 point polygon
  point.x = 2.0;
  point.y = 2.0;
  ASSERT_EQ(true, isInPolygon<geometry_msgs::Point>(polygon, point));
}

TEST_F(TestSuite, normalizeEulerAngle) {
  ASSERT_DOUBLE_EQ(M_PI, normalizeEulerAngle(3 * M_PI));
  ASSERT_DOUBLE_EQ(-M_PI, normalizeEulerAngle(-3 * M_PI));
  ASSERT_DOUBLE_EQ(-M_PI + 0.1, normalizeEulerAngle(M_PI + 0.1));
  ASSERT_DOUBLE_EQ(M_PI - 0.2, normalizeEulerAngle(-M_PI - 0.2));
}

TEST_F(TestSuite, transformToAbsoluteCoordinate2D) {
  geometry_msgs::Point point;
  geometry_msgs::Pose origin;
  geometry_msgs::Point res;

  // no translation and rotation
  res = transformToAbsoluteCoordinate2D(point, origin);
  ASSERT_NEAR(0.0, res.x, ERROR);
  ASSERT_NEAR(0.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // only translation
  point.x = 2.0;
  point.y = 2.0;
  origin.position.x = 5.0;
  origin.position.y = 3.0;

  res = transformToAbsoluteCoordinate2D(point, origin);
  ASSERT_NEAR(7.0, res.x, ERROR);
  ASSERT_NEAR(5.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // translation and rotation 90 deg
  tf2::Quaternion tf_q;
  point.x = 3.0;
  point.y = 2.0;
  origin.position.x = 4.0;
  origin.position.y = 3.0;
  tf_q.setRPY(0.0, 0.0, 90 * M_PI / 180);
  origin.orientation = tf2::toMsg(tf_q);

  res = transformToAbsoluteCoordinate2D(point, origin);
  ASSERT_NEAR(2.0, res.x, ERROR);
  ASSERT_NEAR(6.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // translation and rotation -90 deg
  point.x = 2.0;
  point.y = -3.0;
  origin.position.x = 5.0;
  origin.position.y = -4.0;
  tf_q.setRPY(0.0, 0.0, -90 * M_PI / 180);
  origin.orientation = tf2::toMsg(tf_q);

  res = transformToAbsoluteCoordinate2D(point, origin);
  ASSERT_NEAR(2.0, res.x, ERROR);
  ASSERT_NEAR(-6.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);
}

TEST_F(TestSuite, transformToRelativeCoordinate2D) {
  geometry_msgs::Point point;
  geometry_msgs::Pose origin;
  geometry_msgs::Point res;

  // no translation and rotation
  res = transformToRelativeCoordinate2D(point, origin);
  ASSERT_NEAR(0.0, res.x, ERROR);
  ASSERT_NEAR(0.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // only translation
  point.x = 7.0;
  point.y = 5.0;
  origin.position.x = 5.0;
  origin.position.y = 3.0;
  res = transformToRelativeCoordinate2D(point, origin);
  ASSERT_NEAR(2.0, res.x, ERROR);
  ASSERT_NEAR(2.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // translation and rotation 90 deg
  tf2::Quaternion tf_q;

  point.x = 2.0;
  point.y = 6.0;
  origin.position.x = 4.0;
  origin.position.y = 3.0;
  tf_q.setRPY(0.0, 0.0, 90 * M_PI / 180);
  origin.orientation = tf2::toMsg(tf_q);
  res = transformToRelativeCoordinate2D(point, origin);
  ASSERT_NEAR(3.0, res.x, ERROR);
  ASSERT_NEAR(2.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);

  // translation and rotation -90 deg
  point.x = 2.0;
  point.y = -6.0;
  origin.position.x = 5.0;
  origin.position.y = -4.0;
  tf_q.setRPY(0.0, 0.0, -90 * M_PI / 180);
  origin.orientation = tf2::toMsg(tf_q);
  res = transformToRelativeCoordinate2D(point, origin);
  ASSERT_NEAR(2.0, res.x, ERROR);
  ASSERT_NEAR(-3.0, res.y, ERROR);
  ASSERT_NEAR(0.0, res.z, ERROR);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "TestNode");
  return RUN_ALL_TESTS();
}
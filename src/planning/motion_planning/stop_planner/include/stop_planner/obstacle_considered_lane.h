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

#pragma once

// ROS includes
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

// User defined includes
#include "autoware_planning_msgs/Trajectory.h"
#include "stop_planner/planning_utils.h"
#include "stop_planner/stop_factor_search_utils.h"

// C++ includes
#include <chrono>
#include <memory>

namespace motion_planner
{

// left bottom, left top, right top, right bottom, middle bottom, middle top
typedef std::vector<geometry_msgs::Point> PolygonX;
bool insertExtendWaypoint(const autoware_planning_msgs::Trajectory &lane, const double extend_size, autoware_planning_msgs::Trajectory &output);

class ObstacleConsideredLane
{
public:
  ObstacleConsideredLane()
    : is_obstacle_detected_(false)
    , replan_pair_(std::make_pair(false, autoware_planning_msgs::Trajectory()))
    , stop_distance_(7.0)
    , points_thr_(1)
    , extend_area_size_(5.0)
    , extend_area_size_rev_(2.0)
    , detection_area_width_(1.5)
    , vehicle_length_baselink_to_front_(3.0)
    , vehicle_width_(1.5)
    , search_distance_(50.0)
    , search_distance_rev_(30.0)
    , stop_factor_pose_(geometry_msgs::Pose())
    , actual_stop_pose_(geometry_msgs::Pose())
    , wall_pose_(geometry_msgs::Pose())
    , perpendicular_pose_(geometry_msgs::Pose())
    , lane_in_(autoware_planning_msgs::Trajectory())
    , pc_(sensor_msgs::PointCloud2())
    , polygons_(std::vector<PolygonX>())
  {
  }

  // setter
  void setCurrentLane(const autoware_planning_msgs::Trajectory &lane)
  {
    lane_in_ = lane;
  }

  void setStopDistance(double stop_dist)
  {
    stop_distance_ = stop_dist;
  }

  void setPointsThreshold(int32_t thr)
  {
    points_thr_ = thr;
  }

  void setDetectionAreaWidth(double daw)
  {
    detection_area_width_ = daw;
  }

  void setExtendAreaSize(double eas, double easr)
  {
    extend_area_size_ = eas;
    extend_area_size_rev_ = easr;
  }

  void setBaselinkToFrontLength(double in)
  {
    vehicle_length_baselink_to_front_ = in;
  }

  void setBaselinkToRearLength(double in)
  {
    vehicle_length_baselink_to_rear_ = in;
  }

  void setVehicleWidth(double in)
  {
    vehicle_width_ = in;
  }

  void setPointCloud(const sensor_msgs::PointCloud2 &pc)
  {
    pc_ = pc;
  }

  void setCurrentPose(const geometry_msgs::Pose &curr_pose)
  {
    curr_pose_ = curr_pose;
  }

  void setSearchDistance(double dist, double dist_rev)
  {
    search_distance_ = dist;
    search_distance_rev_ = dist_rev;
  }

  // publish
  void publishStopFactorInfo(ros::Publisher &pub)
  {
    // ROS_DEBUG_STREAM("ObstacleConsideredLane: " << __func__);
    // autoware_planner_msgs::StopFactor sf;
    // sf.header.stamp = ros::Time::now();
    // sf.header.frame_id = "map";
    // sf.point = perpendicular_pose_.position;
    // sf.stp_point = actual_stop_pose_.position;
    // sf.wall_point = wall_pose_.position;
    // sf.kind = stop_kind_;
    // sf.message = "obstacle";
    // pub.publish(sf);
  };

  autoware_planning_msgs::Trajectory run();
  visualization_msgs::MarkerArray visualize();

private:
  void createPolygons();

  std::pair<bool, autoware_planning_msgs::Trajectory> replan(autoware_planning_msgs::Trajectory &trajectory, bool &is_obstacle_detected);
  bool calcVehicleShapeDetectionWidth(const autoware_planning_msgs::Trajectory &lane, const double &shape_tread,
                                      const double &shape_length, const int idx, double &left_y, double &right_y);
  int getBehindLengthClosest(const autoware_planning_msgs::Trajectory &lane, const int start, const double &length);

  // variables
  bool is_obstacle_detected_;
  std::pair<bool, autoware_planning_msgs::Trajectory> replan_pair_;
  double stop_distance_;
  int32_t points_thr_;
  double extend_area_size_;
  double extend_area_size_rev_;
  double detection_area_width_;
  double vehicle_length_baselink_to_front_;
  double vehicle_length_baselink_to_rear_;
  double vehicle_width_;
  double search_distance_;
  double search_distance_rev_;
  geometry_msgs::Pose stop_factor_pose_;
  geometry_msgs::Pose actual_stop_pose_;
  geometry_msgs::Pose wall_pose_;
  geometry_msgs::Pose perpendicular_pose_;
  autoware_planning_msgs::Trajectory lane_in_;
  sensor_msgs::PointCloud2 pc_;
  std::vector<PolygonX> polygons_;
  PolygonX poly_;
  geometry_msgs::Pose curr_pose_;


};

visualization_msgs::Marker displayObstaclePerpendicularPoint(const geometry_msgs::Pose &pose, int8_t kind);
visualization_msgs::Marker displayObstaclePoint(const geometry_msgs::Pose &pose, int8_t kind);
visualization_msgs::MarkerArray displayActiveDetectionArea(const PolygonX &polygons, int8_t kind);

// left, right
std::pair<geometry_msgs::Point, geometry_msgs::Point> calcVehicleFrontPosition(const geometry_msgs::Pose &curr_pose, double base_link_to_front, double width);
std::pair<geometry_msgs::Point, geometry_msgs::Point> calcVehicleRearPosition(const geometry_msgs::Pose &curr_pose, double base_link_to_front, double width);

// direction 0 : front, 1 : reverse
std::tuple<bool, int32_t, int32_t> calcSearchRange(const autoware_planning_msgs::Trajectory &lane, const geometry_msgs::Pose &curr_pose,
                                                   const std::pair<geometry_msgs::Point, geometry_msgs::Point> &edge_pos,
                                                   double search_distance, int8_t direction);
PolygonX createPolygon(const autoware_planning_msgs::Trajectory &lane,
                       const std::vector<double> &left_ofs,
                       const std::vector<double> &right_ofs,
                       int32_t idx_s, int32_t idx_e);

bool findClosestPointPosAndIdx(const autoware_planning_msgs::Trajectory &lane, const std::vector<double> &left_ofs,
                               const std::vector<double> &right_ofs, const std::vector<geometry_msgs::Point> &points,
                               const int32_t idx_s, const int32_t idx_e, int32_t &closest_idx, geometry_msgs::Point &closest_point);

std::pair<bool, geometry_msgs::Pose> calcFopPose(const geometry_msgs::Point &line_s, const geometry_msgs::Point &line_e, geometry_msgs::Point point);


}
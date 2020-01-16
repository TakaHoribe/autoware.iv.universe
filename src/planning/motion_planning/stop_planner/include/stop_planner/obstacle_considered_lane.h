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
bool extendTrajectory(const autoware_planning_msgs::Trajectory &lane, const double extend_length, autoware_planning_msgs::Trajectory &output);

class ObstacleConsideredLane
{
public:
  ObstacleConsideredLane();

  // setter
  void setCurrentTrajectory(const autoware_planning_msgs::Trajectory &traj)
  {
    in_trajectory_ = traj;
  }

  void setStopDistance(double stop_dist)
  {
    planning_param_.stop_distance = stop_dist;
  }

  void setPointsThreshold(int32_t thr)
  {
    planning_param_.points_thr = thr;
  }

  void setDetectionAreaWidth(double daw)
  {
    planning_param_.detection_area_width = daw;
  }

  void setExtendAreaSize(double eas, double easr)
  {
    planning_param_.endpoint_extend_length = eas;
    planning_param_.endpoint_extend_length_rev = easr;
  }

  void setBaselinkToFrontLength(double in)
  {
    vehicle_param_.baselink_to_front_length = in;
  }

  void setBaselinkToRearLength(double in)
  {
    vehicle_param_.baselink_to_rear_length = in;
  }

  void setVehicleWidth(double in)
  {
    vehicle_param_.width = in;
  }

  void setPointCloud(const sensor_msgs::PointCloud2 &pc)
  {
    in_point_cloud_ = pc;
  }

  void setCurrentPose(const geometry_msgs::Pose &curr_pose)
  {
    curr_pose_ = curr_pose;
  }

  void setSearchDistance(double dist, double dist_rev)
  {
    planning_param_.search_distance = dist;
    planning_param_.search_distance_rev = dist_rev;
  }


  autoware_planning_msgs::Trajectory run();
  bool plan(autoware_planning_msgs::Trajectory &out_trajectory);
  visualization_msgs::MarkerArray visualize();

private:

 // variables
  bool is_obstacle_detected_;

  autoware_planning_msgs::Trajectory in_trajectory_;

  sensor_msgs::PointCloud2 in_point_cloud_;
  std::vector<PolygonX> polygons_;
  PolygonX current_detection_area_;
  geometry_msgs::Pose curr_pose_;

  /* vehicle shape parameters */ 
  struct VehicleParam
  {
    double baselink_to_front_length;
    double baselink_to_rear_length;
    double width;
  };
  VehicleParam vehicle_param_;

  /* obstacle detection planning parameters */
  struct PlanningParam
  {
    double stop_distance;
    int32_t points_thr;
    double endpoint_extend_length;
    double endpoint_extend_length_rev;
    double detection_area_width;
    double search_distance;
    double search_distance_rev;
  };
  PlanningParam planning_param_;

  /* visualize */
  geometry_msgs::Pose stop_factor_pose_;
  geometry_msgs::Pose stop_pose_baselink_;
  geometry_msgs::Pose stop_pose_front_;
  geometry_msgs::Pose perpendicular_pose_;


  bool calcDetectionWidthWithVehicleShape(const autoware_planning_msgs::Trajectory &lane, const double &shape_tread,
                                      const double &shape_length, const int idx, double &left_y, double &right_y);
  int getIdxWithBehindLength(const autoware_planning_msgs::Trajectory &lane, const int start, const double &length);
  void calcVehicleEdgePoints(const geometry_msgs::Pose &curr_pose, const VehicleParam &param, std::vector<geometry_msgs::Point> &edge_points);
  bool calcSearchRange(const autoware_planning_msgs::Trajectory &lane, const geometry_msgs::Pose &curr_pose,
                       const int8_t direction, int32_t &start_idx, int32_t &end_idx);
  PolygonX createPolygon(const autoware_planning_msgs::Trajectory &lane, const std::vector<double> &left_ofs,
                         const std::vector<double> &right_ofs, int32_t idx_s, int32_t idx_e);
  bool findClosestPointPosAndIdx(const autoware_planning_msgs::Trajectory &lane, const std::vector<double> &left_ofs,
                                const std::vector<double> &right_ofs, const std::vector<geometry_msgs::Point> &points,
                                const int32_t idx_s, const int32_t idx_e, int32_t &closest_idx, geometry_msgs::Point &closest_point);

};




}
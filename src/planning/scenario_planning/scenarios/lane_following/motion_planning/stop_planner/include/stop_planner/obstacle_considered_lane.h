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
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// User defined includes
#include "autoware_planning_msgs/Trajectory.h"
#include "stop_planner/planning_utils.h"

// C++ includes
#include <chrono>
#include <memory>

namespace motion_planner
{

geometry_msgs::Pose getPoseOnTrajectoryWithDistance(const autoware_planning_msgs::Trajectory &in_tajectory, const geometry_msgs::Point &origin,
                                                    const int start_idx, const double distance);

// left bottom, left top, right top, right bottom, middle bottom, middle top
typedef std::vector<geometry_msgs::Point> PolygonX;

class ObstacleConsideredLane
{
public:
  ObstacleConsideredLane();

  // setter
  void setCurrentTrajectory(const autoware_planning_msgs::Trajectory &traj) { in_trajectory_ = traj; }
  void setPointCloud(const sensor_msgs::PointCloud2 &pc) { in_point_cloud_ = pc; }
  void setCurrentPose(const geometry_msgs::Pose &curr_pose) { curr_pose_ = curr_pose; }
  void setStopDistance(double stop_dist) { planning_param_.stop_distance = stop_dist; }
  void setPointsThreshold(int32_t thr) { planning_param_.points_thr = thr; }
  void setDetectionAreaWidth(double daw) { planning_param_.detection_area_width = daw; }
  void setBaselinkToFrontLength(double in) { vehicle_param_.baselink_to_front_length = in; }
  void setBaselinkToRearLength(double in) { vehicle_param_.baselink_to_rear_length = in; }
  void setVehicleWidth(double in) { vehicle_param_.width = in; }
  void setSearchDistance(double dist, double dist_rev)
  {
    planning_param_.search_distance = dist;
    planning_param_.search_distance_rev = dist_rev;
  }
  void setExtendAreaSize(double eas, double easr)
  {
    planning_param_.endpoint_extend_length = eas;
    planning_param_.endpoint_extend_length_rev = easr;
  }
  void getExtructedPcd(sensor_msgs::PointCloud2 &pcd) { pcd = pcd_around_traj_; };

  autoware_planning_msgs::Trajectory run();
  visualization_msgs::MarkerArray visualize();

private:
  // variables
  bool is_obstacle_detected_;

  autoware_planning_msgs::Trajectory in_trajectory_;
  sensor_msgs::PointCloud2 in_point_cloud_;
  sensor_msgs::PointCloud2 pcd_around_traj_;
  PolygonX current_detection_area_;
  geometry_msgs::Pose curr_pose_;

  enum class Direction
  {
    FORWARD = 0,
    REVERSE = 1,
    INVALID = 2,
  };
  Direction current_direction_;

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
  geometry_msgs::Pose obstacle_pose_;
  geometry_msgs::Pose stop_pose_baselink_;
  geometry_msgs::Pose stop_pose_front_;

  bool resampleTrajectory(const autoware_planning_msgs::Trajectory &in_trajectory, const double resample_interval_dist,
                          autoware_planning_msgs::Trajectory &out);
  bool calcStopPose(const autoware_planning_msgs::Trajectory &in_trajectory, const pcl::PointCloud<pcl::PointXYZ> &in_pcd,
                         geometry_msgs::Pose &stop_pose, bool &is_obstacle_detected);
  bool overwriteStopVelocity(const autoware_planning_msgs::Trajectory &in_trajectory, const geometry_msgs::Pose &stop_pose,
                             autoware_planning_msgs::Trajectory &out);
  bool extractPcdAroundTrajectory2D(const pcl::PointCloud<pcl::PointXYZ> &in_pcd, const autoware_planning_msgs::Trajectory &in_traj,
                                    double radius, /*out=*/pcl::PointCloud<pcl::PointXYZ> &out_pcd);


  Direction calcTrajectoryDirection(const autoware_planning_msgs::Trajectory &trajectory, double dist_thr = 0.05);

  bool calcDetectionWidthWithVehicleShape(const autoware_planning_msgs::Trajectory &lane, const double &shape_tread,
                                          const double &shape_length, const int idx, double &left_y, double &right_y);
  void calcVehicleEdgePoints(const geometry_msgs::Pose &curr_pose, const VehicleParam &param, std::vector<geometry_msgs::Point> &edge_points);
  bool calcSearchRangeIdx(const autoware_planning_msgs::Trajectory &lane, const geometry_msgs::Pose &curr_pose,
                          int32_t &start_idx, int32_t &end_idx);
  PolygonX createPolygon(const autoware_planning_msgs::Trajectory &lane, const std::vector<double> &left_ofs,
                         const std::vector<double> &right_ofs, int32_t idx_s, int32_t idx_e);
  bool calcClosestPointInPolygon(const autoware_planning_msgs::Trajectory &lane, const std::vector<double> &left_ofs,
                                 const std::vector<double> &right_ofs, const std::vector<geometry_msgs::Point> &points,
                                 const int32_t idx_s, const int32_t idx_e, int32_t &closest_idx, geometry_msgs::Pose &closest_pose);
  geometry_msgs::Pose calcClosestPointProjectedOnTrajectory(const std::vector<geometry_msgs::Point> &points,
                                                            const autoware_planning_msgs::Trajectory &in_traj,
                                                            const int origin_idx);
  autoware_planning_msgs::Trajectory extendTrajectory(const autoware_planning_msgs::Trajectory &in_trajectory);

  std::pair<bool, int32_t> calcForwardIdxByLineIntegral(const autoware_planning_msgs::Trajectory &lane, int32_t base_idx, double stop_offset_dist) const;
  bool findPointsInPolygon(const PolygonX &poly, const std::vector<geometry_msgs::Point> &points, const int32_t points_thr, std::vector<geometry_msgs::Point> &out_points) const;
};

} // namespace motion_planner
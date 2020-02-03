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

#include "stop_planner/obstacle_pcd_velocity_planner.h"
#include "stop_planner/visualize.h"

ObstaclePcdVelocityPlanner::ObstaclePcdVelocityPlanner()
{
  vehicle_param_.baselink_to_front_length = 3.0;
  vehicle_param_.baselink_to_rear_length = 1.0;
  vehicle_param_.width = 2.5;

  planning_param_.stop_distance = 7.0;
  planning_param_.points_thr = 1;
  planning_param_.endpoint_extend_length = 5.0;
  planning_param_.endpoint_extend_length_rev = 5.0;
  planning_param_.detection_area_width = 1.5;
  planning_param_.search_distance = 50.0;
  planning_param_.search_distance_rev = 30.0;
}

autoware_planning_msgs::Trajectory ObstaclePcdVelocityPlanner::run()
{
  /* check Trajectory Direction */
  current_direction_ = calcTrajectoryDirection(in_trajectory_);
  if (current_direction_ == Direction::INVALID)
  {
    ROS_DEBUG("[stop] invalid direction. please check trajectory.");
    return in_trajectory_; // no obstacle. return raw trajectory.
  }

  /* extend trajectory for endpoint detection */
  autoware_planning_msgs::Trajectory extended_traj;
  extendTrajectory(in_trajectory_, extended_traj);

  /* resample trajectory to rough distance for computational cost */
  double resample_dist = 1.0;
  autoware_planning_msgs::Trajectory resampled_traj;
  resampleTrajectory(extended_traj, resample_dist, /*out=*/resampled_traj);

  pcl::PointCloud<pcl::PointXYZ> in_pcl_pcd;
  pcl::fromROSMsg(in_point_cloud_, in_pcl_pcd);

  /* Rough downsample of pointcloud around the object trajectory */
  const double radius = std::max(3.0, planning_param_.detection_area_width * 1.414);
  pcl::PointCloud<pcl::PointXYZ> pcd_around_traj;
  extractPcdAroundTrajectory2D(in_pcl_pcd, resampled_traj, radius, /*out=*/pcd_around_traj);
  pcl::toROSMsg(pcd_around_traj, pcd_around_traj_);

  /* calculate stop position */
  geometry_msgs::Pose stop_pose;
  if (!calcStopPoseOnTrajectory(resampled_traj, pcd_around_traj, /*out=*/stop_pose, is_obstacle_detected_))
  {
    ROS_ERROR_DELAYED_THROTTLE(1.0, "[stop] fail to calculate stop position.");
    return in_trajectory_; // no obstacle. return raw trajectory.
  }
  if (!is_obstacle_detected_)
  {
    ROS_DEBUG("[stop] no obstacle");
    return in_trajectory_; // no obstacle. return raw trajectory.
  }

  /* set stop velocity from stop point */
  autoware_planning_msgs::Trajectory out_trajectory;
  planStopVelocity(in_trajectory_, stop_pose, /*out=*/out_trajectory);

  return out_trajectory;
}

bool ObstaclePcdVelocityPlanner::extractPcdAroundTrajectory2D(const pcl::PointCloud<pcl::PointXYZ> &in_pcd, const autoware_planning_msgs::Trajectory &in_traj,
                                                              double radius, /*out=*/pcl::PointCloud<pcl::PointXYZ> &out_pcd)
{
  out_pcd.clear();

  double squared_radius = radius * radius;
  int in_pcd_max = in_pcd.size();
  int in_traj_max = in_traj.points.size();
  for (int i = 0; i < in_pcd_max; ++i) // for each pcd
  {
    for (int j = 0; j < in_traj_max; ++j) // for each trajectory point
    {
      const double &dx = in_pcd[i].x - in_traj.points[j].pose.position.x;
      const double &dy = in_pcd[i].y - in_traj.points[j].pose.position.y;
      const double &squared_dist2d = dx * dx + dy * dy;
      if (squared_dist2d < squared_radius)
      {
        out_pcd.push_back(in_pcd.at(i));
        break;
      }
    }
  }
}

bool ObstaclePcdVelocityPlanner::planStopVelocity(const autoware_planning_msgs::Trajectory &in_trajectory,
                                                  const geometry_msgs::Pose &stop_pose, autoware_planning_msgs::Trajectory &out)
{
  out = in_trajectory;

  int stop_idx = -1;
  if (!planning_utils::findClosestIdxWithDistAngThr(in_trajectory, stop_pose, stop_idx))
  {
    ROS_WARN_DELAYED_THROTTLE(1.0, "cannot get closest at planStopVelocity().");
    return false; // cannot get closest error
  }

  for (unsigned int k = stop_idx + 1; k < out.points.size(); k++)
  {
    out.points.at(k).twist.linear.x = 0.0;
  }

  /* insert stop point for more accurate stop position */
  // autoware_planning_msgs::TrajectoryPoint tp;
  // tp.pose = stop_pose;
  // tp.twist.linear.x = 0.0;
  // out.points.insert(out.points.begin() + stop_idx + 1, tp);

  return true;
}

bool ObstaclePcdVelocityPlanner::resampleTrajectory(const autoware_planning_msgs::Trajectory &in_trajectory, const double resample_interval_dist,
                                                    autoware_planning_msgs::Trajectory &out)
{
  if (in_trajectory.points.size() == 0)
  {
    ROS_DEBUG("traj.size = 0");
    out = in_trajectory;
    return true;
  }

  std::vector<double> dist_arr;
  double dist_sum = 0.0;
  dist_arr.push_back(dist_sum);
  for (int i = 0; i < in_trajectory.points.size() - 1; ++i)
  {
    double ds = planning_utils::calcDistance2D(in_trajectory.points.at(i).pose.position, in_trajectory.points.at(i + 1).pose.position);
    dist_sum += ds;
    dist_arr.push_back(dist_sum);
  }

  std::vector<double> interp_dist_arr;
  double interp_dist_sim = 0.0;
  for (double ds = 0; ds < dist_arr.back(); ds += resample_interval_dist)
  {
    interp_dist_arr.push_back(ds);
  }

  if (!planning_utils::linearInterpTrajectory(dist_arr, in_trajectory, interp_dist_arr, out))
  {
    ROS_WARN("[stop planer] trajectory interpolation error");
    return false;
  }

  ROS_DEBUG("[resample] before: %lu, after: %lu", in_trajectory.points.size(), out.points.size());

  return true;
}

bool ObstaclePcdVelocityPlanner::calcStopPoseOnTrajectory(const autoware_planning_msgs::Trajectory &in_trajectory, const pcl::PointCloud<pcl::PointXYZ> &in_pcd,
                                                          geometry_msgs::Pose &stop_pose, bool &is_obstacle_detected)
{
  is_obstacle_detected = false;

  // Guard
  if (planning_param_.points_thr == 0 || in_trajectory.points.empty())
  {
    ROS_DEBUG("no plan needed. points_thr = %d, pcd.size = %lu, trajectory size = %lu", planning_param_.points_thr, in_pcd.size(), in_trajectory.points.size());
    return true;
  }

  int current_closest;
  if (!planning_utils::findClosestIdxWithDistAngThr(in_trajectory, current_pose_, /*out=*/current_closest))
  {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[stop planner] cannot get closest.");
    return false; // cannot get closest error 
  }

  // define search range
  const double start_dist = 0.0;
  const double end_dist = current_direction_ == Direction::FORWARD ? planning_param_.search_distance : planning_param_.search_distance_rev;
  int start_idx = planning_utils::calcForwardIdxByLineIntegral(in_trajectory, current_closest, start_dist);
  int end_idx = planning_utils::calcForwardIdxByLineIntegral(in_trajectory, current_closest, end_dist);

  // create the whole detection area (only for visualize)
  std::vector<double> left_width_v, right_width_v;
  calcDetectionWidthWithVehicleShape(in_trajectory, left_width_v, right_width_v);
  current_detection_area_ = createPolygonFromTrajectory(in_trajectory, left_width_v, right_width_v, start_idx, end_idx);


  // calculate closest pcd point with interpolation
  int32_t closest;
  if (!calcClosestPointInPolygon(in_trajectory, in_pcd, start_idx, end_idx, closest, obstacle_pose_))
  {
    ROS_DEBUG("[StopPlanner] no point is in polygon in binary search. return input trajectory as it is.");
    return true;
  }

  // calculate stop point index for base_link
  const double vehicle_length = (current_direction_ == Direction::FORWARD) ? (vehicle_param_.baselink_to_front_length)
                                                                           : (vehicle_param_.baselink_to_rear_length);
  const double stop_dist_for_baselink = vehicle_length + planning_param_.stop_distance;
  stop_pose = planning_utils::getPoseOnTrajectoryWithRadius(in_trajectory, obstacle_pose_.position,
                                                            closest, (-1.0) * stop_dist_for_baselink /* search for behind */);
  stop_pose_front_ = planning_utils::getPoseOnTrajectoryWithRadius(in_trajectory, obstacle_pose_.position,
                                                                   closest, (-1.0) * planning_param_.stop_distance /* search for behind */);

  is_obstacle_detected = true;
  return true;
}

bool ObstaclePcdVelocityPlanner::calcDetectionWidthWithVehicleShape(const autoware_planning_msgs::Trajectory &in_trajectory,
                                                                    std::vector<double> &left_widths, std::vector<double> &right_widths)
{
  left_widths.clear();
  right_widths.clear();

  const double shape_tread = vehicle_param_.width;
  const double shape_length = vehicle_param_.baselink_to_front_length;

  geometry_msgs::Point p_fl_shape; // front left point in local coordinate
  p_fl_shape.x = shape_length;
  p_fl_shape.y = shape_tread * 0.5;

  geometry_msgs::Point p_fr_shape; // front right point in local coordinate
  p_fr_shape.x = shape_length;
  p_fr_shape.y = -shape_tread * 0.5;

  const double left_base = planning_param_.detection_area_width * 0.5;
  const double right_base = planning_param_.detection_area_width * 0.5;

  for (int i = 0; i < in_trajectory.points.size(); ++i)
  {
    const geometry_msgs::Pose base_p = in_trajectory.points.at(i).pose;
    const geometry_msgs::Pose p_i = planning_utils::getPoseOnTrajectoryWithRadius(in_trajectory, base_p.position, i, (-1.0) * shape_length);

    // calculat vehicle edge point (froint_right/left) when vehicle is behind with baselink_to_front length
    const geometry_msgs::Point p_fl_g = planning_utils::transformToAbsoluteCoordinate2D(p_fl_shape, p_i);
    const geometry_msgs::Point p_fr_g = planning_utils::transformToAbsoluteCoordinate2D(p_fr_shape, p_i);
    const geometry_msgs::Point p_fl_l = planning_utils::transformToRelativeCoordinate2D(p_fl_g, base_p);
    const geometry_msgs::Point p_fr_l = planning_utils::transformToRelativeCoordinate2D(p_fr_g, base_p);
    const double alpha_l = std::atan2(p_fl_l.y, p_fl_l.x);
    const double alpha_r = std::atan2(p_fr_l.y, p_fr_l.x);
    const double dw_l = planning_utils::calcDistance2D(p_fl_g, base_p.position) * std::sin(alpha_l);
    const double dw_r = planning_utils::calcDistance2D(p_fr_g, base_p.position) * (-1.0) * std::sin(alpha_r);

    const double left = std::max(left_base, dw_l);
    const double right = std::max(right_base, dw_r);

    left_widths.push_back(left);
    right_widths.push_back(right);
  }

  return true;
}

void ObstaclePcdVelocityPlanner::calcVehicleEdgePoints(const geometry_msgs::Pose &curr_pose, const VehicleParam &param,
                                                       std::vector<geometry_msgs::Point> &edge_points)
{
  edge_points.clear();
  const double half_width = param.width / 2.0;

  geometry_msgs::Pose front_left;
  front_left.position.x += param.baselink_to_front_length;
  front_left.position.y += half_width;
  edge_points.push_back(planning_utils::transformToAbsoluteCoordinate2D(front_left.position, curr_pose));

  geometry_msgs::Pose front_right;
  front_right.position.x += param.baselink_to_front_length;
  front_right.position.y -= half_width;
  edge_points.push_back(planning_utils::transformToAbsoluteCoordinate2D(front_right.position, curr_pose));

  geometry_msgs::Pose rear_left;
  rear_left.position.x -= param.baselink_to_rear_length;
  rear_left.position.y += half_width;
  edge_points.push_back(planning_utils::transformToAbsoluteCoordinate2D(rear_left.position, curr_pose));

  geometry_msgs::Pose rear_right;
  rear_right.position.x -= param.baselink_to_rear_length;
  rear_right.position.y -= half_width;
  edge_points.push_back(planning_utils::transformToAbsoluteCoordinate2D(rear_right.position, curr_pose));

  return;
}

bool ObstaclePcdVelocityPlanner::extendTrajectory(const autoware_planning_msgs::Trajectory &input, autoware_planning_msgs::Trajectory &output)
{
  output = input;
  if (input.points.empty())
  {
    return true;
  }

  geometry_msgs::Pose ext_pose;
  geometry_msgs::Point ext_p;
  ext_p.x = (current_direction_ == Direction::FORWARD) ? planning_param_.endpoint_extend_length
                                                       : (-1.0) * planning_param_.endpoint_extend_length_rev;

  ext_pose.position = planning_utils::transformToAbsoluteCoordinate2D(ext_p, input.points.back().pose);
  ext_pose.orientation = input.points.back().pose.orientation;

  output.points.push_back(output.points.back());
  output.points.back().pose = ext_pose;

  return true;
}

PolygonX ObstaclePcdVelocityPlanner::createPolygonFromTrajectory(const autoware_planning_msgs::Trajectory &in_trajectory, const std::vector<double> &left_width,
                                                                 const std::vector<double> &right_width, int32_t start_idx, int32_t end_idx)
{
  PolygonX poly;

  // insert left
  for (int32_t i = start_idx; i <= end_idx; i++)
  {
    geometry_msgs::Point edge_width;
    edge_width.y = left_width.at(i);
    geometry_msgs::Point edge_width_abs = planning_utils::transformToAbsoluteCoordinate2D(edge_width, in_trajectory.points.at(i).pose);
    poly.push_back(edge_width_abs);
  }

  // insert right
  for (int32_t i = end_idx; i >= start_idx; i--)
  {
    geometry_msgs::Point edge_width;
    edge_width.y = -right_width.at(i);
    geometry_msgs::Point edge_width_abs = planning_utils::transformToAbsoluteCoordinate2D(edge_width, in_trajectory.points.at(i).pose);
    poly.push_back(edge_width_abs);
  }

  return poly;
}

bool ObstaclePcdVelocityPlanner::calcClosestPoseProjectedOnTrajectory(const pcl::PointCloud<pcl::PointXYZ> &in_pcd,
                                                                      const autoware_planning_msgs::Trajectory &in_traj,
                                                                      const int origin_idx, geometry_msgs::Pose &out_pose)
{
  geometry_msgs::Pose origin = in_traj.points.at(origin_idx).pose;

  // convert origin yaw to forward direction.
  if (origin_idx < in_traj.points.size() - 1)
  {
    double dx = in_traj.points.at(origin_idx + 1).pose.position.x - in_traj.points.at(origin_idx).pose.position.x;
    double dy = in_traj.points.at(origin_idx + 1).pose.position.y - in_traj.points.at(origin_idx).pose.position.y;
    if (std::fabs(dx) > 1.0e-5 || std::fabs(dy) > 1.0e-5)
    {
      double yaw = std::atan2(dy, dx);
      origin.orientation = planning_utils::getQuaternionFromYaw(yaw);
    }
  }

  // find closest point with minimum x length on origin coordinates.
  double min_x = std::numeric_limits<double>::max();
  for (const auto &p : in_pcd)
  {
    geometry_msgs::Point geo_p;
    geo_p.x = p.x;
    geo_p.y = p.y;
    geo_p.z = p.z;
    geometry_msgs::Point rel_p = planning_utils::transformToRelativeCoordinate2D(geo_p, origin);
    if (rel_p.x < min_x)
    {
      min_x = rel_p.x;
    }
  }

  // project closest point on the trajectory
  out_pose = origin;
  double yaw = tf2::getYaw(origin.orientation);
  out_pose.position.x += min_x * std::cos(yaw);
  out_pose.position.y += min_x * std::sin(yaw);

  return true;
}

bool ObstaclePcdVelocityPlanner::calcClosestPointInPolygon(const autoware_planning_msgs::Trajectory &in_trajectory, const pcl::PointCloud<pcl::PointXYZ> &in_pcd,
                                                           const int32_t start_idx, const int32_t end_idx, int32_t &closest_idx, geometry_msgs::Pose &closest_pose)
{
  if (start_idx >= end_idx)
  {
    closest_idx = -1;
    closest_pose = geometry_msgs::Pose();
    if (start_idx > end_idx)
      ROS_WARN("[calcClosestPointInPolygon] undesired index. s = %d, e = %d", start_idx, end_idx);
    return false;
  }

  // calculate polygon width for each trajectory points with vehicle shape.
  std::vector<double> left_width_v, right_width_v;
  calcDetectionWidthWithVehicleShape(in_trajectory, left_width_v, right_width_v);

  // create the whole detection area (only for visualize)
  current_detection_area_ = createPolygonFromTrajectory(in_trajectory, left_width_v, right_width_v, start_idx, end_idx);

  // search closest points by bilinear search
  int32_t start_idx_loop = start_idx;
  int32_t end_idx_loop = end_idx;
  pcl::PointCloud<pcl::PointXYZ> points_in_polygon;
  while (true)
  {
    const int32_t idx_m_loop = (int32_t)((end_idx_loop + start_idx_loop) / 2);

    PolygonX front_polygon = createPolygonFromTrajectory(in_trajectory, left_width_v, right_width_v, start_idx_loop, idx_m_loop);

    // is points in front polygon?
    if (extructPointsInPolygon(front_polygon, in_pcd, 1, points_in_polygon))
    {
      if ((idx_m_loop - start_idx_loop) == 1)
      {
        calcClosestPoseProjectedOnTrajectory(points_in_polygon, in_trajectory, start_idx_loop, closest_pose);
        closest_idx = start_idx_loop;
        return true;
      }

      end_idx_loop = idx_m_loop;
      continue;
    }

    PolygonX rear_polygon = createPolygonFromTrajectory(in_trajectory, left_width_v, right_width_v, idx_m_loop, end_idx_loop);

    // is points in rear polygon?
    if (extructPointsInPolygon(rear_polygon, in_pcd, 1, points_in_polygon))
    {
      if ((end_idx_loop - idx_m_loop) == 1)
      {
        calcClosestPoseProjectedOnTrajectory(points_in_polygon, in_trajectory, idx_m_loop, closest_pose);
        closest_idx = idx_m_loop;
        return true;
      }

      start_idx_loop = idx_m_loop;
      continue;
    }

    // no obstacle found.
    closest_idx = -1;
    closest_pose = geometry_msgs::Pose();
    return false;
  }
}

bool ObstaclePcdVelocityPlanner::extructPointsInPolygon(const PolygonX &poly, const pcl::PointCloud<pcl::PointXYZ> &in_pcd,
                                                        const int32_t points_thr, pcl::PointCloud<pcl::PointXYZ> &out_pcd)
{
  out_pcd.clear();
  for (const auto &p : in_pcd)
  {
    geometry_msgs::Point geo_p;
    geo_p.x = p.x;
    geo_p.y = p.y;
    geo_p.z = p.z;
    if (planning_utils::isInPolygon(poly, geo_p))
    {
      out_pcd.push_back(p);
    }
  }

  if ((int32_t)out_pcd.size() >= points_thr)
  {
    return true;
  }
  else
  {
    return false;
  }
}

ObstaclePcdVelocityPlanner::Direction ObstaclePcdVelocityPlanner::calcTrajectoryDirection(const autoware_planning_msgs::Trajectory &trajectory, double dist_thr)
{
  if (trajectory.points.size() < 2)
  {
    return Direction::INVALID;
  }

  for (uint32_t i = 0; i < trajectory.points.size(); i++)
  {
    geometry_msgs::Pose prev;
    geometry_msgs::Pose next;

    if (i == (trajectory.points.size() - 1))
    {
      prev = trajectory.points.at(i - 1).pose;
      next = trajectory.points.at(i).pose;
    }
    else
    {
      prev = trajectory.points.at(i).pose;
      next = trajectory.points.at(i + 1).pose;
    };

    if (planning_utils::calcDistSquared2D(prev.position, next.position) > dist_thr * dist_thr)
    {
      const geometry_msgs::Point rel_p = planning_utils::transformToRelativeCoordinate2D(next.position, prev);
      return (rel_p.x > 0.0) ? Direction::FORWARD : Direction::REVERSE;
    }
  }

  ROS_ERROR("lane is something wrong. poses.size = %lu, dist_thr = %f", trajectory.points.size(), dist_thr);
  return Direction::INVALID;
}

visualization_msgs::MarkerArray ObstaclePcdVelocityPlanner::visualize()
{
  visualization_msgs::MarkerArray ma;

  const int8_t stop_kind = is_obstacle_detected_ ? 1 /* obstacle */ : 0 /* none */;
  const visualization_msgs::MarkerArray ma_d = displayActiveDetectionArea(current_detection_area_, stop_kind);
  ma.markers.insert(ma.markers.end(), ma_d.markers.begin(), ma_d.markers.end());

  if (is_obstacle_detected_)
  {
    ma.markers.push_back(displayWall(stop_pose_front_, stop_kind, 1));
    ma.markers.push_back(displayObstaclePerpendicularPoint(obstacle_pose_, stop_kind));
  }
  ma.markers.push_back(displayObstaclePoint(obstacle_pose_, stop_kind));
  return ma;
}
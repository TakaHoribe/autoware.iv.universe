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

#include "stop_planner/obstacle_considered_lane.h"
#include "stop_planner/visualize.h"

namespace motion_planner
{

ObstacleConsideredLane::ObstacleConsideredLane() : perpendicular_pose_(geometry_msgs::Pose()), in_trajectory_(autoware_planning_msgs::Trajectory()),
                                                   in_point_cloud_(sensor_msgs::PointCloud2())
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

autoware_planning_msgs::Trajectory ObstacleConsideredLane::run()
{
  /* check Trajectory Direction */
  current_direction_ = calcTrajectoryDirection(in_trajectory_);
  if (current_direction_ == Direction::INVALID)
  {
    ROS_INFO("[stop] invalid direction. please check trajectory.");
    return in_trajectory_; // no obstacle. return raw trajectory.
  }

  /* extend trajectory for endpoint detection */
  autoware_planning_msgs::Trajectory extended_traj = extendTrajectory(in_trajectory_);

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
  calcStopPose(resampled_traj, pcd_around_traj, /*out=*/stop_pose, is_obstacle_detected_);
  if (!is_obstacle_detected_)
  {
    ROS_INFO("[stop] no obstacle");
    return in_trajectory_; // no obstacle. return raw trajectory.
  }

  /* set stop velocity from stop point */
  autoware_planning_msgs::Trajectory out_trajectory;
  overwriteStopVelocity(in_trajectory_, stop_pose, /*out=*/out_trajectory);

  return out_trajectory;
}

bool ObstacleConsideredLane::extractPcdAroundTrajectory2D(const pcl::PointCloud<pcl::PointXYZ> &in_pcd, const autoware_planning_msgs::Trajectory &in_traj,
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

bool ObstacleConsideredLane::overwriteStopVelocity(const autoware_planning_msgs::Trajectory &in_trajectory,
                                                   const geometry_msgs::Pose &stop_pose, autoware_planning_msgs::Trajectory &out)
{
  out = in_trajectory;

  int stop_idx = -1;
  if (!planning_utils::findClosestIdxWithDistAngThr(in_trajectory, stop_pose, stop_idx))
  {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[stop planner] cannot get closest at overwriteStopVelocity().");
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

bool ObstacleConsideredLane::resampleTrajectory(const autoware_planning_msgs::Trajectory &in_trajectory, const double resample_interval_dist,
                                                autoware_planning_msgs::Trajectory &out)
{
  if (in_trajectory.points.size() == 0)
  {
    ROS_INFO("traj.size = 0");
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

  ROS_INFO("[resample] before: %lu, after: %lu", in_trajectory.points.size(), out.points.size());

  return true;
}

bool ObstacleConsideredLane::calcStopPose(const autoware_planning_msgs::Trajectory &in_trajectory, const pcl::PointCloud<pcl::PointXYZ> &in_pcd,
                                               geometry_msgs::Pose &stop_pose, bool &is_obstacle_detected)
{
  ROS_DEBUG_STREAM(__func__);

  is_obstacle_detected = false;

  // Guard
  if (in_pcd.empty() || planning_param_.points_thr == 0 || in_trajectory.points.empty())
  {
    ROS_INFO("no plan needed. points_thr = %d, pcd.size = %lu, trajectory size = %lu", planning_param_.points_thr, in_pcd.size(), in_trajectory.points.size());
    return true;
  }

  // define search range
  int32_t idx_start, idx_end;
  if (!calcSearchRangeIdx(in_trajectory, curr_pose_, idx_start, idx_end))
  {
    ROS_WARN("cannot get search range");
    return false;
  }
  ROS_DEBUG("start: %d, end: %d, pointcloud: %lu", idx_start, idx_end, in_pcd.size());

  // 車両形状を考慮して、ポリゴン作成のための横幅を計算する
  std::vector<double> left_width_v(in_trajectory.points.size(), 0.0);
  std::vector<double> right_width_v(in_trajectory.points.size(), 0.0);
  for (int32_t i = 0; i < (int32_t)in_trajectory.points.size(); i++)
  {
    calcDetectionWidthWithVehicleShape(in_trajectory, planning_param_.detection_area_width, vehicle_param_.baselink_to_front_length, i,
                                       left_width_v.at(i), right_width_v.at(i));
  }
  ROS_DEBUG("left: %lu, right: %lu", left_width_v.size(), right_width_v.size());

  // 経路と左右の幅のvectorから経路を囲う大きなポリゴンを作る。（可視化用）
  current_detection_area_ = createPolygon(in_trajectory, left_width_v, right_width_v, idx_start, idx_end);

  // 検出エリアに入っている点群を抽出する
  std::vector<geometry_msgs::Point> pcd_in_polygon;
  for (size_t i = 0; i < in_pcd.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = in_pcd.at(i).x;
    p.y = in_pcd.at(i).y;
    p.z = in_pcd.at(i).z;
    pcd_in_polygon.push_back(p);
  }

  // 二分探索で点群の存在するもっとも近いポリゴンのidxと、点の位置を求める
  int32_t closest;
  if (!calcClosestPointInPolygon(in_trajectory, left_width_v, right_width_v, pcd_in_polygon, idx_start, idx_end, closest, obstacle_pose_))
  {
    ROS_DEBUG("[StopPlanner] no point is in polygon in binary search. return input trajectory as it is.");
    return false;
  }

  // calculate stop point index for base_link
  const auto stop_dist_baselink = (current_direction_ == Direction::FORWARD) ? (planning_param_.stop_distance + vehicle_param_.baselink_to_front_length)
                                                                              : (planning_param_.stop_distance + vehicle_param_.baselink_to_rear_length);
  
  stop_pose = getPoseOnTrajectoryWithDistance(in_trajectory, obstacle_pose.position, closest, (-1.0) * stop_dist_baselink /* search for behind */);
  stop_pose_front_ = getPoseOnTrajectoryWithDistance(in_trajectory, obstacle_pose.position, closest, (-1.0) * planning_param_.stop_distance /* search for behind */);


  is_obstacle_detected = true;
  return true;
}

geometry_msgs::Pose getPoseOnTrajectoryWithDistance(const autoware_planning_msgs::Trajectory &in_trajectory, const geometry_msgs::Point &origin,
                                                    const int start_idx, const double distance)
{
  if (start_idx < 0 || start_idx >= in_trajectory.points.size())
  {
    return geometry_msgs::Pose();
  }

  double prev_dist = 0.0;
  geometry_msgs::Point p_prev = origin;
  if (distance > 0)
  {
    for (uint i = start_idx; i < in_trajectory.points.size() - 1; ++i)
    {
      geometry_msgs::Point p_i = in_trajectory.points.at(i).pose.position;
      double dist_i = planning_utils::calcDistance2D(p_i, origin);
      if (dist_i > distance)
      {
        double d_next = dist_i - distance;
        double d_prev = distance - prev_dist;
        double d_all = std::max(dist_i - prev_dist, 1.0E-5 /* avoid 0 divide */);
        geometry_msgs::Pose p;
        p.position.x = (d_next * p_prev.x + d_prev * p_i.x) / d_all;
        p.position.y = (d_next * p_prev.y + d_prev * p_i.y) / d_all;
        p.position.z = (d_next * p_prev.z + d_prev * p_i.z) / d_all;
        p.orientation = in_trajectory.points.at(i).pose.orientation; // TODO : better to do interpolation by yaw 
        return p;
      }
      prev_dist = dist_i;
      p_prev = p_i;
    }
    return in_trajectory.points.back().pose;
  }
  else if (distance < 0)
  {
    double abs_distance = std::fabs(distance);
    for (uint i = start_idx; i > 0; --i)
    {
      geometry_msgs::Point p_i = in_trajectory.points.at(i).pose.position;
      double dist_i = planning_utils::calcDistance2D(p_i, origin);
      if (dist_i > abs_distance)
      {
        double d_next = dist_i - abs_distance;
        double d_prev = abs_distance - prev_dist;
        double d_all = std::max(dist_i - prev_dist, 1.0E-5 /* avoid 0 divide */);
        geometry_msgs::Pose p;
        p.position.x = (d_next * p_prev.x + d_prev * p_i.x) / d_all;
        p.position.y = (d_next * p_prev.y + d_prev * p_i.y) / d_all;
        p.position.z = (d_next * p_prev.z + d_prev * p_i.z) / d_all;
        p.orientation = in_trajectory.points.at(i).pose.orientation; // TODO : better to do interpolation by yaw 
        return p;
      }
      prev_dist = dist_i;
      p_prev = p_i;
    }
    return in_trajectory.points.front().pose;
  }
  else
  {
    return in_trajectory.points.at(start_idx).pose;
  }  

}

bool ObstacleConsideredLane::calcDetectionWidthWithVehicleShape(const autoware_planning_msgs::Trajectory &in_trajectory, const double &shape_tread,
                                                                const double &shape_length, const int idx, double &left_y,
                                                                double &right_y)
{
  left_y = shape_tread * 0.5;
  right_y = shape_tread * 0.5;

  geometry_msgs::Point p_fl_shape; // front left point in local coordinate
  p_fl_shape.x = shape_length;
  p_fl_shape.y = shape_tread * 0.5;

  geometry_msgs::Point p_fr_shape; // front right point in local coordinate
  p_fr_shape.x = shape_length;
  p_fr_shape.y = -shape_tread * 0.5;

  const geometry_msgs::Pose base_p = in_trajectory.points.at(idx).pose;
  const geometry_msgs::Pose p_i = getPoseOnTrajectoryWithDistance(in_trajectory, base_p.position, idx, (-1.0) * shape_length);

  // calculat vehicle edge point (froint_right/left) when vehicle is behind with baselink_to_front length
  const geometry_msgs::Point p_fl_g = planning_utils::transformToAbsoluteCoordinate2D(p_fl_shape, p_i);
  const geometry_msgs::Point p_fr_g = planning_utils::transformToAbsoluteCoordinate2D(p_fr_shape, p_i);
  const geometry_msgs::Point p_fl_l = planning_utils::transformToRelativeCoordinate2D(p_fl_g, base_p);
  const geometry_msgs::Point p_fr_l = planning_utils::transformToRelativeCoordinate2D(p_fr_g, base_p);
  const double alpha_l = std::atan2(p_fl_l.y, p_fl_l.x);
  const double alpha_r = std::atan2(p_fr_l.y, p_fr_l.x);
  const double dw_l = planning_utils::calcDistance2D(p_fl_g, base_p.position) * std::sin(alpha_l);
  const double dw_r = planning_utils::calcDistance2D(p_fr_g, base_p.position) * (-1.0) * std::sin(alpha_r);

  left_y = std::max(left_y, dw_l);
  right_y = std::max(right_y, dw_r);

  return true;
}

void ObstacleConsideredLane::calcVehicleEdgePoints(const geometry_msgs::Pose &curr_pose, const VehicleParam &param,
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

bool ObstacleConsideredLane::calcSearchRangeIdx(const autoware_planning_msgs::Trajectory &in_trajectory, const geometry_msgs::Pose &curr_pose,
                                                 int32_t &start_idx, int32_t &end_idx)
{
  std::vector<geometry_msgs::Point> edge_points;
  calcVehicleEdgePoints(curr_pose, vehicle_param_, edge_points);
  geometry_msgs::Point left, right;
  double search_distance;
  if (current_direction_ == Direction::FORWARD)
  {
    left = edge_points.at(0);  // front left
    right = edge_points.at(1); // front right
    search_distance = planning_param_.search_distance;
  }
  else if (current_direction_ == Direction::REVERSE)
  {
    left = edge_points.at(2);  // rear left
    right = edge_points.at(3); // rear right
    search_distance = planning_param_.search_distance_rev;
  }


  start_idx = -1;
  end_idx = -1;

  int closest_idx = -1;
  if (!planning_utils::findClosestIdxWithDistAngThr(in_trajectory, curr_pose, /*out*/closest_idx, 3.0, M_PI_4))
  {
    ROS_WARN("Cannot find closest");
    return false;
  }


  // search start and end index
  for (uint32_t i = closest_idx; i < in_trajectory.points.size(); i++)
  {
    const geometry_msgs::Pose e = in_trajectory.points.at(i).pose;
    auto rel_left = planning_utils::transformToRelativeCoordinate2D(left, e);
    auto rel_right = planning_utils::transformToRelativeCoordinate2D(right, e);

    ROS_DEBUG("rel_left: %lf, %lf", rel_left.x, rel_left.y);
    ROS_DEBUG("rel_right: %lf, %lf", rel_right.x, rel_right.y);

    if ((i == (in_trajectory.points.size() - 1)) || ((current_direction_ == Direction::FORWARD) && (rel_left.x < 0 || rel_right.x < 0)) || ((current_direction_ == Direction::REVERSE) && (rel_left.x > 0 || rel_right.x > 0))) // front
    {
      start_idx = std::max((int)i - 1, 0);
      end_idx = calcForwardIdxByLineIntegral(in_trajectory, i, search_distance).second;
      return true;
    }
  }

  ROS_ERROR("cannot find closest: reach end");
  return false;
}

autoware_planning_msgs::Trajectory ObstacleConsideredLane::extendTrajectory(const autoware_planning_msgs::Trajectory &in_trajectory)
{

  if (in_trajectory.points.empty())
  {
    return in_trajectory;
  }

  autoware_planning_msgs::Trajectory out = in_trajectory;

  geometry_msgs::Pose ext_pose;
  geometry_msgs::Point ext_p;
  ext_p.x = (current_direction_ == Direction::FORWARD) ? planning_param_.endpoint_extend_length
                                                              : (-1.0) * planning_param_.endpoint_extend_length_rev;

  ext_pose.position = planning_utils::transformToAbsoluteCoordinate2D(ext_p, in_trajectory.points.back().pose);
  ext_pose.orientation = in_trajectory.points.back().pose.orientation;

  out.points.push_back(out.points.back());
  out.points.back().pose = ext_pose;

  return out;
}

PolygonX ObstacleConsideredLane::createPolygon(const autoware_planning_msgs::Trajectory &in_trajectory, const std::vector<double> &left_ofs,
                                               const std::vector<double> &right_ofs, int32_t idx_s, int32_t idx_e)
{
  ROS_DEBUG_STREAM(__func__);
  PolygonX poly;

  // insert left
  for (int32_t i = idx_s; i <= idx_e; i++)
  {
    geometry_msgs::Point ofs;
    ofs.y = left_ofs.at(i);
    auto ofs_abs = planning_utils::transformToAbsoluteCoordinate2D(ofs, in_trajectory.points.at(i).pose);
    poly.push_back(ofs_abs);
  }

  // insert right
  for (int32_t i = idx_e; i >= idx_s; i--)
  {
    geometry_msgs::Point ofs;
    ofs.y = -right_ofs.at(i);
    auto ofs_abs = planning_utils::transformToAbsoluteCoordinate2D(ofs, in_trajectory.points.at(i).pose);
    poly.push_back(ofs_abs);
  }

  return poly;
}

geometry_msgs::Pose ObstacleConsideredLane::calcClosestPointProjectedOnTrajectory(const std::vector<geometry_msgs::Point> &points,
                                                                                  const autoware_planning_msgs::Trajectory &in_traj,
                                                                                  const int origin_idx)
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
  geometry_msgs::Point clst_p;
  double min_x = std::numeric_limits<double>::max();
  for (const auto &p : points)
  {
    auto rel_p = planning_utils::transformToRelativeCoordinate2D(p, origin);
    if (rel_p.x < min_x)
    {
      clst_p = p;
      min_x = rel_p.x;
    }
  }

  // project closest point on the trajectory
  geometry_msgs::Pose projected_pose = origin;
  double yaw = tf2::getYaw(origin.orientation);
  projected_pose.position.x += min_x * std::cos(yaw); 
  projected_pose.position.y += min_x * std::sin(yaw); 

  return projected_pose;
}

bool ObstacleConsideredLane::calcClosestPointInPolygon(const autoware_planning_msgs::Trajectory &in_trajectory, const std::vector<double> &left_ofs,
                                                       const std::vector<double> &right_ofs, const std::vector<geometry_msgs::Point> &points,
                                                       const int32_t idx_s, const int32_t idx_e, int32_t &closest_idx, geometry_msgs::Pose &closest_pose)
{
  if (idx_s >= idx_e)
  {
    closest_idx = -1;
    closest_pose = geometry_msgs::Pose();
    return false;
  }

  int32_t idx_s_loop = idx_s;
  int32_t idx_e_loop = idx_e;
  while (true)
  {
    const int32_t idx_m_loop = (int32_t)((idx_e_loop + idx_s_loop) / 2);

    auto poly1 = createPolygon(in_trajectory, left_ofs, right_ofs, idx_s_loop, idx_m_loop);

    // is points in polygon?
    std::vector<geometry_msgs::Point> points_in_poly1;
    const bool is_points_in_poly1 = findPointsInPolygon(poly1, points, 1, points_in_poly1);

    if (is_points_in_poly1)
    {
      if ((idx_m_loop - idx_s_loop) == 1)
      {
        closest_pose = calcClosestPointProjectedOnTrajectory(points_in_poly1, in_trajectory, idx_s_loop);
        closest_idx = idx_s_loop;
        return true;
      }

      idx_e_loop = idx_m_loop;
      continue;
    }

    auto poly2 = createPolygon(in_trajectory, left_ofs, right_ofs, idx_m_loop, idx_e_loop);

    std::vector<geometry_msgs::Point> points_in_poly2;
    const bool is_points_in_poly2 = findPointsInPolygon(poly2, points, 1, points_in_poly2);
    if (is_points_in_poly2)
    {
      if ((idx_e_loop - idx_m_loop) == 1)
      {
        closest_pose = calcClosestPointProjectedOnTrajectory(points_in_poly2, in_trajectory, idx_m_loop);
        closest_idx = idx_m_loop;
        return true;
      }

      idx_s_loop = idx_m_loop;
      continue;
    }

    // ROS_ERROR("nothing is polygon");
    closest_idx = -1;
    closest_pose = geometry_msgs::Pose();
    return false;
  }
}

visualization_msgs::MarkerArray ObstacleConsideredLane::visualize()
{
  ROS_DEBUG_STREAM(__func__);
  visualization_msgs::MarkerArray ma;

  const int8_t stop_kind = is_obstacle_detected_ ? 1 /* obstacle */ : 0 /* none */;
  const auto ma_d = displayActiveDetectionArea(current_detection_area_, stop_kind);
  ma.markers.insert(ma.markers.end(), ma_d.markers.begin(), ma_d.markers.end());

  if (is_obstacle_detected_)
  {
    ma.markers.push_back(displayWall(stop_pose_front_, stop_kind, 1));
    ma.markers.push_back(displayObstaclePerpendicularPoint(obstacle_pose_, stop_kind));
  }
  ma.markers.push_back(displayObstaclePoint(obstacle_pose_, stop_kind));

  return ma;
}

std::pair<bool, int32_t> ObstacleConsideredLane::calcForwardIdxByLineIntegral(const autoware_planning_msgs::Trajectory &in_trajectory,
                                                                              int32_t base_idx, double stop_offset_dist) const
{
  ROS_DEBUG_STREAM(__func__);

  if (in_trajectory.points.empty() || (base_idx < 0) || base_idx > (int32_t)(in_trajectory.points.size() - 1))
    return std::make_pair(false, -1);

  int32_t actual_idx = base_idx;
  double accum = 0.0;
  while (actual_idx < (int32_t)in_trajectory.points.size())
  {
    if (actual_idx == (int32_t)in_trajectory.points.size() - 1)
    {
      ROS_DEBUG("idx: %d, accum: %lf, dist: %lf", actual_idx, accum, stop_offset_dist);
      return std::make_pair(true, actual_idx);
    }

    accum += planning_utils::calcDistance2D(in_trajectory.points.at(actual_idx + 1).pose.position,
                                            in_trajectory.points.at(actual_idx).pose.position);

    if (accum > stop_offset_dist)
    {
      ROS_DEBUG("idx: %d, accum: %lf, dist: %lf", actual_idx, accum, stop_offset_dist);
      return std::make_pair(true, actual_idx + 1);
    }

    actual_idx++;
  }

  return std::make_pair(false, -1);
}

bool ObstacleConsideredLane::findPointsInPolygon(const PolygonX &poly, const std::vector<geometry_msgs::Point> &points,
                                                 const int32_t points_thr, std::vector<geometry_msgs::Point> &out_points) const
{
  out_points.clear();
  for (const auto &p : points)
  {
    if (planning_utils::isInPolygon(poly, p))
    {
      out_points.push_back(p);
    }
  }

  if ((int32_t)out_points.size() >= points_thr)
  {
    return true;
  }
  else
  {
    return false;
  }
}

ObstacleConsideredLane::Direction ObstacleConsideredLane::calcTrajectoryDirection(const autoware_planning_msgs::Trajectory &trajectory, double dist_thr)
{
  if(trajectory.points.size() < 2)
  {
    ROS_ERROR("size of points is smaller than 2");
    return Direction::INVALID;
  }

  for(uint32_t i = 0; i < trajectory.points.size(); i++)
  {
    geometry_msgs::Pose prev;
    geometry_msgs::Pose next;

    if(i == (trajectory.points.size() - 1))
    {
      prev = trajectory.points.at(i - 1).pose;
      next = trajectory.points.at(i).pose;
    }
    else
    {
      prev = trajectory.points.at(i).pose;
      next = trajectory.points.at(i + 1).pose;
    };

    if(planning_utils::calcDistSquared2D(prev.position, next.position) > dist_thr * dist_thr)
    {
      const auto rel_p = planning_utils::transformToRelativeCoordinate2D(next.position, prev);
      return (rel_p.x > 0.0) ? Direction::FORWARD : Direction::REVERSE;
    }
  }

  ROS_ERROR("lane is something wrong. poses.size = %lu, dist_thr = %f", trajectory.points.size(), dist_thr);
  return Direction::INVALID;
}

} // namespace motion_planner
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
  /* resample trajectory to rough distance for computational cost */
  double resample_dist = 1.0;
  autoware_planning_msgs::Trajectory resampled_traj;
  resampleTrajectory(in_trajectory_, resample_dist, /*out=*/resampled_traj);

  pcl::PointCloud<pcl::PointXYZ> in_pcl_pcd;
  pcl::fromROSMsg(in_point_cloud_, in_pcl_pcd);

  /* Rough downsample of pointcloud around the object trajectory */
  const double radius = 2.0;
  pcl::PointCloud<pcl::PointXYZ> pcd_around_traj;
  extractPcdAroundTrajectory2D(in_pcl_pcd, resampled_traj, radius, /*out=*/pcd_around_traj);
  pcl::toROSMsg(pcd_around_traj, pcd_around_traj_);
  

  /* calculate stop position */
  geometry_msgs::Pose stop_pose;
  calculateStopPose(resampled_traj, pcd_around_traj, /*out=*/stop_pose, is_obstacle_detected_);
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

  std::vector<geometry_msgs::Pose> poses = planning_utils::extractPoses(in_trajectory);
  std::pair<bool, int32_t> ret = planning_utils::findClosestIdxWithDistAngThr(poses, stop_pose);
  if (!ret.first)
  {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[stop planner] cannot get closest at overwriteStopVelocity().");
    return false; // cannot get closest error
  }
  int stop_idx = (int)ret.second;

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

bool ObstacleConsideredLane::calculateStopPose(const autoware_planning_msgs::Trajectory &in_trajectory, const pcl::PointCloud<pcl::PointXYZ> &in_pcd,
                                               geometry_msgs::Pose &stop_pose, bool &is_obstacle_detected)
{
  ROS_DEBUG_STREAM(__func__);

  bool ret = false;

  is_obstacle_detected = false;

  // Guard
  if (in_pcd.empty() || planning_param_.points_thr == 0)
  {
    ROS_INFO("point cloud is not subscribed or empty. points_thr = %d, pcd.size = %lu", planning_param_.points_thr, in_pcd.size());
    return true;
  }
  if (in_trajectory.points.empty())
  {
    ROS_INFO("trajectory size is 0. no plan.");
    return true;
  }

  // get lane direction (0:FWD, 1:REV, 2:ERROR)
  const int8_t direction = planning_utils::getLaneDirection(planning_utils::extractPoses(in_trajectory));
  if (direction != 0 && direction != 1)
  {
    ROS_WARN("cannot get lane direction!");
    return false;
  }

  // insert extend search area
  double endpoint_extend_length = direction == 0 ? planning_param_.endpoint_extend_length : planning_param_.endpoint_extend_length_rev;

  autoware_planning_msgs::Trajectory extended_trajectory;
  ret = extendTrajectory(in_trajectory, endpoint_extend_length, extended_trajectory);
  if (!ret)
  {
    ROS_WARN("cannot extend lane");
    return false;
  }

  // define search range
  int32_t idx_start, idx_end;
  ret = calcSearchRangeIdx(extended_trajectory, curr_pose_, direction, idx_start, idx_end);
  if (!ret)
  {
    ROS_WARN("cannot get search range");
    return false;
  }
  ROS_DEBUG("start: %d, end: %d, pointcloud: %lu", idx_start, idx_end, in_pcd.size());

  // 車両形状を考慮して、ポリゴン作成のための横幅を計算する
  std::vector<double> left_width_v(extended_trajectory.points.size(), 0.0);
  std::vector<double> right_width_v(extended_trajectory.points.size(), 0.0);
  for (int32_t i = 0; i < (int32_t)extended_trajectory.points.size(); i++)
  {
    calcDetectionWidthWithVehicleShape(extended_trajectory, planning_param_.detection_area_width, vehicle_param_.baselink_to_front_length, i,
                                       left_width_v.at(i), right_width_v.at(i));
  }
  ROS_DEBUG("left: %lu, right: %lu", left_width_v.size(), right_width_v.size());

  // 経路と左右の幅のvectorから経路を囲う大きなポリゴンを作る。
  current_detection_area_ = createPolygon(extended_trajectory, left_width_v, right_width_v, idx_start, idx_end);

  // 検出エリアに入っている点群を抽出する
  std::vector<geometry_msgs::Point> pcd_in_polygon;
  // ret = findPointCloudInPolygon(current_detection_area_, in_pcd, planning_param_.points_thr, pcd_in_polygon);
  // if (!ret)
  // {
  //   ROS_DEBUG("[StopPlanner] no point is in polygon. return input trajectory as it is.");
  //   return true;
  // }

  for (size_t i = 0; i < in_pcd.size(); ++i)
  {
    geometry_msgs::Point p;
    p.x = in_pcd.at(i).x;
    p.y = in_pcd.at(i).y;
    p.z = in_pcd.at(i).z;
    pcd_in_polygon.push_back(p);
  }

  // 二分探索で点群の存在するもっとも近いポリゴンのidxと、点の位置を求める
  int32_t clst_idx;
  geometry_msgs::Point clst_point;
  ret = findClosestPointPosAndIdx(extended_trajectory, left_width_v, right_width_v, pcd_in_polygon, idx_start, idx_end, clst_idx, clst_point);
  if (!ret)
  {
    ROS_DEBUG("[StopPlanner] no point is in polygon in binary search. return input trajectory as it is.");
    return false;
  }
  ROS_DEBUG("closest point before: %d, (%lf, %lf)", clst_idx, clst_point.x, clst_point.y);

  const auto pnt_start = extended_trajectory.points.at(clst_idx).pose.position;
  const auto pnt_end = extended_trajectory.points.at(clst_idx + 1).pose.position;

  // calculate stop_factor_pose (the foot of perpendicular line)
  const auto fop_pair = calcFopPose(pnt_start, pnt_end, clst_point);
  if (!fop_pair.first)
    return false;
  perpendicular_pose_ = fop_pair.second;

  stop_factor_pose_.position = clst_point;

  // calculate stop point index for base_link
  const auto stop_dist_baselink = (direction == 0) ? (planning_param_.stop_distance + vehicle_param_.baselink_to_front_length)
                                                   : (planning_param_.stop_distance + vehicle_param_.baselink_to_rear_length);
  const auto asp_tuple = planning_utils::calcDistanceConsideredPoseAndIdx(extended_trajectory, perpendicular_pose_, clst_idx, stop_dist_baselink, 1);
  if (!std::get<0>(asp_tuple))
    return false;
  const auto stop_idx = std::get<1>(asp_tuple);
  stop_pose = std::get<2>(asp_tuple); // for base_link

  // calculate stop point index for vehicle front
  const auto wall_tuple = planning_utils::calcDistanceConsideredPoseAndIdx(extended_trajectory, perpendicular_pose_, clst_idx, planning_param_.stop_distance, 1);
  if (!std::get<0>(wall_tuple))
    return false;
  stop_pose_front_ = std::get<2>(wall_tuple);

  is_obstacle_detected = true;
  return true;
}

int ObstacleConsideredLane::getIdxWithBehindLength(const autoware_planning_msgs::Trajectory &lane, const int start,
                                                   const double &length)
{
  if (start < 0 || start > (int)lane.points.size() - 1)
  {
    ROS_ERROR("[obstacle_considered_lane] : getIdxWithBehindLength() : bad index!");
    return start;
  }
  double err_min = 1.0E10;
  int idx_min = -1;
  for (int i = start; i >= 0; --i)
  {
    const double dist = planning_utils::calcDistance2D(lane.points.at(start).pose.position,
                                                       lane.points.at(i).pose.position);
    const double err = std::fabs(dist - length);
    if (err >= err_min)
    {
      break;
    }
    if (err < err_min)
    {
      err_min = err;
      idx_min = i;
    }
  }
  if (idx_min == -1)
  {
    ROS_ERROR("[obstacle_considered_lane] : getIdxWithBehindLength() : bad index");
    return start;
  }
  else
  {
    return idx_min;
  }
}

bool ObstacleConsideredLane::calcDetectionWidthWithVehicleShape(const autoware_planning_msgs::Trajectory &lane, const double &shape_tread,
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

  const geometry_msgs::Pose base_p = lane.points.at(idx).pose;

  int idx_target = getIdxWithBehindLength(lane, idx, shape_length); // calc waypoint behind baselink_to_front length
  const geometry_msgs::Pose p_i = lane.points.at(idx_target).pose;

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

  /* debug */
  // const double rad2deg = 180.0 / 3.14159265;
  // double yaw0 = tf2::getYaw(lane.points.at(idx_target).pose.orientation) * rad2deg;
  // double yaw1 = tf2::getYaw(lane.points.at(idx).pose.orientation) * rad2deg;
  // printf("idx = %d, N = %d, p_fl_shape = [%f, %f], p_fr_shape = [%f, %f], yaw_0 = %f, yaw_1 = %f, yaw_diff = %f\n", idx,
  //        idx - idx_target, p_fl_shape.x, p_fl_shape.y, p_fr_shape.x, p_fr_shape.y, yaw0, yaw1, yaw1 - yaw0);
  // printf("L_l = %f, alpha_l = %f, dw_l = %f, left = %f ||  L_r = %f, alpha_r = %f, dw_r = %f, right = %f\n",
  //       planning_utils::calcDistance2D(p_fl_g, base_p.position), alpha_l *rad2deg, dw_l, left_y,
  //       planning_utils::calcDistance2D(p_fr_g, base_p.position), alpha_r *rad2deg, dw_r, right_y);

  return true;
}

void ObstacleConsideredLane::calcVehicleEdgePoints(const geometry_msgs::Pose &curr_pose, const VehicleParam &param, std::vector<geometry_msgs::Point> &edge_points)
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

bool ObstacleConsideredLane::calcSearchRangeIdx(const autoware_planning_msgs::Trajectory &lane, const geometry_msgs::Pose &curr_pose,
                                                const int8_t direction, int32_t &start_idx, int32_t &end_idx)
{
  std::vector<geometry_msgs::Point> edge_points;
  calcVehicleEdgePoints(curr_pose, vehicle_param_, edge_points);
  geometry_msgs::Point left, right;
  double search_distance;
  if (direction == 0)
  {
    left = edge_points.at(0);  // front left
    right = edge_points.at(1); // front right
    search_distance = planning_param_.search_distance;
  }
  else if (direction == 1)
  {
    left = edge_points.at(2);  // rear left
    right = edge_points.at(3); // rear right
    search_distance = planning_param_.search_distance_rev;
  }

  auto clst_pair =
      planning_utils::findClosestIdxWithDistAngThr(planning_utils::extractPoses(lane), curr_pose, 3.0, M_PI_4);

  start_idx = -1;
  end_idx = -1;

  if (!clst_pair.first)
  {
    ROS_WARN("Cannot find closest");
    return false;
  }

  ROS_DEBUG("clst idx: %d", clst_pair.second);

  // search start and end index
  for (uint32_t i = clst_pair.second; i < lane.points.size(); i++)
  {
    const geometry_msgs::Pose e = lane.points.at(i).pose;
    auto rel_left = planning_utils::transformToRelativeCoordinate2D(left, e);
    auto rel_right = planning_utils::transformToRelativeCoordinate2D(right, e);

    ROS_DEBUG("rel_left: %lf, %lf", rel_left.x, rel_left.y);
    ROS_DEBUG("rel_right: %lf, %lf", rel_right.x, rel_right.y);

    if ((i == (lane.points.size() - 1)) || ((direction == 0) && (rel_left.x < 0 || rel_right.x < 0)) || ((direction == 1) && (rel_left.x > 0 || rel_right.x > 0))) // front
    {
      start_idx = std::max((int)i - 1, 0);
      end_idx = calcForwardIdxByLineIntegral(lane, i, search_distance).second;
      return true;
    }
  }

  ROS_ERROR("cannot find closest: reach end");
  return false;
}

bool extendTrajectory(const autoware_planning_msgs::Trajectory &lane, const double extend_length, autoware_planning_msgs::Trajectory &out)
{
  if (lane.points.empty())
  {
    ROS_ERROR("lane is empty");
    return false;
  }

  geometry_msgs::Pose ext_pose;

  const auto direction = planning_utils::getLaneDirection(planning_utils::extractPoses(lane));
  if (direction == 2)
    return false;

  geometry_msgs::Point ext_p;
  ext_p.x = direction == 0 ? extend_length : (-1.0) * extend_length;

  ext_pose.position = planning_utils::transformToAbsoluteCoordinate2D(ext_p, lane.points.back().pose);
  ext_pose.orientation = lane.points.back().pose.orientation;

  out = lane;
  out.points.push_back(out.points.back());
  out.points.back().pose = ext_pose;

  return true;
}

PolygonX ObstacleConsideredLane::createPolygon(const autoware_planning_msgs::Trajectory &lane, const std::vector<double> &left_ofs,
                                               const std::vector<double> &right_ofs, int32_t idx_s, int32_t idx_e)
{
  ROS_DEBUG_STREAM(__func__);
  PolygonX poly;

  // insert left
  for (int32_t i = idx_s; i <= idx_e; i++)
  {
    geometry_msgs::Point ofs;
    ofs.y = left_ofs.at(i);
    auto ofs_abs = planning_utils::transformToAbsoluteCoordinate2D(ofs, lane.points.at(i).pose);
    poly.push_back(ofs_abs);
  }

  // insert right
  for (int32_t i = idx_e; i >= idx_s; i--)
  {
    geometry_msgs::Point ofs;
    ofs.y = -right_ofs.at(i);
    auto ofs_abs = planning_utils::transformToAbsoluteCoordinate2D(ofs, lane.points.at(i).pose);
    poly.push_back(ofs_abs);
  }

  return poly;
}

bool ObstacleConsideredLane::findClosestPointPosAndIdx(const autoware_planning_msgs::Trajectory &lane, const std::vector<double> &left_ofs,
                                                       const std::vector<double> &right_ofs, const std::vector<geometry_msgs::Point> &points,
                                                       const int32_t idx_s, const int32_t idx_e, int32_t &closest_idx, geometry_msgs::Point &closest_point)
{
  ROS_DEBUG_STREAM(__FUNCTION__);
  if (idx_s >= idx_e)
  {
    ROS_ERROR("idx_s: %d is larger than idx_e: %d", idx_s, idx_e);
    closest_idx = -1;
    closest_point = geometry_msgs::Point();
    return false;
  }

  int32_t idx_s_loop = idx_s;
  int32_t idx_e_loop = idx_e;
  while (true)
  {
    const int32_t idx_m_loop = (int32_t)((idx_e_loop + idx_s_loop) / 2);
    ROS_DEBUG("idx_s: %d, idx_m: %d, idx_e: %d", idx_s_loop, idx_m_loop, idx_e_loop);

    ROS_DEBUG("create polygon1");
    auto poly1 = createPolygon(lane, left_ofs, right_ofs, idx_s_loop, idx_m_loop);

    // is points in polygon?
    ROS_DEBUG("points_pair1");
    std::vector<geometry_msgs::Point> points_in_poly1;
    const bool is_points_in_poly1 = findPointsInPolygon(poly1, points, 1, points_in_poly1);

    if (is_points_in_poly1)
    {
      if ((idx_m_loop - idx_s_loop) == 1)
      {
        ROS_DEBUG("last polygon1");
        auto clst_p = calcClosestPointByXAxis(lane.points.at(idx_s_loop).pose, points_in_poly1);
        ROS_DEBUG("end: idx: %d, clst_p: (%lf, %lf)", idx_s_loop, clst_p.x, clst_p.y);
        closest_idx = idx_s_loop;
        closest_point = clst_p;
        return true;
      }

      ROS_DEBUG("select poly1, continue");
      idx_e_loop = idx_m_loop;
      continue;
    }

    ROS_DEBUG("create polygon2");
    auto poly2 = createPolygon(lane, left_ofs, right_ofs, idx_m_loop, idx_e_loop);

    ROS_DEBUG("points_pair2");
    std::vector<geometry_msgs::Point> points_in_poly2;
    const bool is_points_in_poly2 = findPointsInPolygon(poly2, points, 1, points_in_poly2);
    if (is_points_in_poly2)
    {
      if ((idx_e_loop - idx_m_loop) == 1)
      {
        ROS_DEBUG("last polygon2");
        auto clst_p = calcClosestPointByXAxis(lane.points.at(idx_m_loop).pose, points_in_poly2);
        ROS_DEBUG("end: idx: %d, clst_p: (%lf, %lf)", idx_m_loop, clst_p.x, clst_p.y);
        closest_idx = idx_m_loop;
        closest_point = clst_p;
        return true;
      }

      ROS_DEBUG("select poly2, continue");
      idx_s_loop = idx_m_loop;
      continue;
    }

    ROS_ERROR("nothing is polygon");
    closest_idx = -1;
    closest_point = geometry_msgs::Point();
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
    ma.markers.push_back(displayObstaclePerpendicularPoint(perpendicular_pose_, stop_kind));
    ma.markers.push_back(displayObstaclePoint(stop_factor_pose_, stop_kind));
  }

  return ma;
}

std::pair<bool, int32_t> ObstacleConsideredLane::calcForwardIdxByLineIntegral(const autoware_planning_msgs::Trajectory &lane,
                                                                              int32_t base_idx, double stop_offset_dist) const
{
  ROS_DEBUG_STREAM(__func__);

  if (lane.points.empty() || (base_idx < 0) || base_idx > (int32_t)(lane.points.size() - 1))
    return std::make_pair(false, -1);

  int32_t actual_idx = base_idx;
  double accum = 0.0;
  while (actual_idx < (int32_t)lane.points.size())
  {
    if (actual_idx == (int32_t)lane.points.size() - 1)
    {
      ROS_DEBUG("idx: %d, accum: %lf, dist: %lf", actual_idx, accum, stop_offset_dist);
      return std::make_pair(true, actual_idx);
    }

    accum += planning_utils::calcDistance2D(lane.points.at(actual_idx + 1).pose.position,
                                            lane.points.at(actual_idx).pose.position);

    if (accum > stop_offset_dist)
    {
      ROS_DEBUG("idx: %d, accum: %lf, dist: %lf", actual_idx, accum, stop_offset_dist);
      return std::make_pair(true, actual_idx + 1);
    }

    actual_idx++;
  }

  return std::make_pair(false, -1);
}

bool ObstacleConsideredLane::findPointCloudInPolygon(const PolygonX &poly, const sensor_msgs::PointCloud2 &pc,
                                                     const int32_t points_thr, std::vector<geometry_msgs::Point> &out_pcd) const
{
  out_pcd.clear();
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc, "x"), iter_y(pc, "y"), iter_z(pc, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
  {
    if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z))
    {
      ROS_DEBUG("rejected for nan in point(%f, %f, %f)\n", *iter_x, *iter_y, *iter_z);
      continue;
    }

    geometry_msgs::Point p;
    p.x = *iter_x;
    p.y = *iter_y;

    if (planning_utils::isInPolygon(poly, p))
    {
      out_pcd.push_back(p);
    }
  }
  ROS_DEBUG("detected, p_count: %lu", out_pcd.size());

  if ((int32_t)out_pcd.size() >= points_thr)
  {
    return true;
  }
  else
  {
    return false;
  }
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
  ROS_DEBUG("detected, p_count: %lu", out_points.size());

  if ((int32_t)out_points.size() >= points_thr)
  {
    return true;
  }
  else
  {
    return false;
  }
}

geometry_msgs::Point ObstacleConsideredLane::calcClosestPointByXAxis(const geometry_msgs::Pose &pose, const std::vector<geometry_msgs::Point> &points) const
{
  geometry_msgs::Point clst_p;
  auto min_x = std::numeric_limits<double>::max();
  for (const auto &p : points)
  {
    auto rel_p = planning_utils::transformToRelativeCoordinate2D(p, pose);
    if (fabs(rel_p.x) < min_x)
    {
      clst_p = p;
      min_x = fabs(rel_p.x);
    }
  }

  return clst_p;
}

std::pair<bool, geometry_msgs::Pose> ObstacleConsideredLane::calcFopPose(const geometry_msgs::Point &line_s, const geometry_msgs::Point &line_e,
                                                                         geometry_msgs::Point point) const
{
  auto fop_pair = planning_utils::calcFootOfPerpendicular(line_s, line_e, point);
  if (!fop_pair.first)
  {
    ROS_ERROR("calcFootOfPerpendicular: cannot calc");
    return std::make_pair(false, geometry_msgs::Pose());
  }

  geometry_msgs::Pose res;
  res.position = fop_pair.second;
  res.orientation = planning_utils::getQuaternionFromYaw(atan2((line_e.y - line_s.y), (line_e.x - line_s.x)));
  ROS_DEBUG("fop: (%lf, %lf)", res.position.x, res.position.y);

  return std::make_pair(true, res);
}

} // namespace motion_planner
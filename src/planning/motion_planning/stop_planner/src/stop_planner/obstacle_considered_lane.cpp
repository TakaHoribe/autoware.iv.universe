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

namespace motion_planner
{
autoware_planning_msgs::Trajectory ObstacleConsideredLane::run()
{
  replan_pair_ = replan();
  is_obstacle_detected_ = replan_pair_.first;
  return replan_pair_.second;
}

visualization_msgs::MarkerArray ObstacleConsideredLane::visualize()
{
  ROS_DEBUG_STREAM(__func__);
  visualization_msgs::MarkerArray ma;

  const int8_t stop_kind = is_obstacle_detected_ ? 1 /* obstacle */ : 0 /* none */;
  ma.markers.push_back(displayWall(wall_pose_, stop_kind, 1));
  ma.markers.push_back(displayObstaclePerpendicularPoint(perpendicular_pose_, stop_kind));
  ma.markers.push_back(displayObstaclePoint(stop_factor_pose_, stop_kind));
  const auto ma_d = displayActiveDetectionArea(poly_, stop_kind);
  ma.markers.insert(ma.markers.end(), ma_d.markers.begin(), ma_d.markers.end());

  return ma;
}

std::pair<bool, autoware_planning_msgs::Trajectory> ObstacleConsideredLane::replan()
{
  ROS_DEBUG_STREAM(__func__);
  autoware_planning_msgs::Trajectory out = lane_in_;

  auto NotReplan = [&out, this]() -> std::pair<bool, autoware_planning_msgs::Trajectory> {
    stop_factor_pose_ = geometry_msgs::Pose();
    actual_stop_pose_ = geometry_msgs::Pose();
    wall_pose_ = geometry_msgs::Pose();
    out.header.stamp = ros::Time::now();
    return std::make_pair(false, out);
  };

  auto Replan = [&out]() -> std::pair<bool, autoware_planning_msgs::Trajectory> {
    out.header.stamp = ros::Time::now();
    return std::make_pair(true, out);
  };


  // get lane direction (0:FWD, 1:REV, 2:ERROR)
  const int8_t direction = planning_utils::getLaneDirection(planning_utils::extractPoses(lane_in_));

  // insert extend search area
  double extend_area_size;
  if(direction == 0) //front
  {
    extend_area_size = extend_area_size_;
  }
  else if(direction == 1)
  {
    extend_area_size = extend_area_size_rev_;
  }
  else
  {
    ROS_WARN("cannot get lane direction!");
    return NotReplan();
  }


  autoware_planning_msgs::Trajectory ext_lane;
  if(!insertExtendWaypoint(lane_in_, extend_area_size_rev_, ext_lane))
  {
    ROS_WARN("cannot extend lane");
    return NotReplan();
  }


  // define search range
  std::tuple<bool, int32_t, int32_t> range_tuple;

  if(direction == 0) // front
  {
    ROS_DEBUG("front");
    auto fp_pair = calcVehicleFrontPosition(curr_pose_, vehicle_length_baselink_to_front_, vehicle_width_);

    range_tuple = calcSearchRange(ext_lane, curr_pose_, fp_pair, search_distance_, direction);
    if (!std::get<0>(range_tuple))
    {
      ROS_WARN("cannot get search range!");
      return NotReplan();
    }
  }
  else if(direction == 1) // reverse
  {
    ROS_DEBUG("reverse");
    auto rp_pair = calcVehicleRearPosition(curr_pose_, vehicle_length_baselink_to_rear_, vehicle_width_);

    range_tuple = calcSearchRange(ext_lane, curr_pose_, rp_pair, search_distance_rev_, direction);
    if (!std::get<0>(range_tuple))
    {
      ROS_WARN("cannot get search range");
      return NotReplan();
    }
  }
  else
  {
    ROS_WARN("cannot get lane direction");
    return NotReplan();
  }

  const auto idx_s = std::get<1>(range_tuple);
  const auto idx_e = std::get<2>(range_tuple);
  ROS_DEBUG("start: %d, end: %d, pointcloud: %d", std::get<1>(range_tuple), std::get<2>(range_tuple),
            (int32_t)pc_.data.size());


  // for polygon
  std::vector<double> left_offset_v(ext_lane.points.size(), 0.0);
  std::vector<double> right_offset_v(ext_lane.points.size(), 0.0);
  for (int32_t i = 0; i < (int32_t)ext_lane.points.size(); i++)
  {
    calcVehicleShapeDetectionWidth(ext_lane, detection_area_width_, vehicle_length_baselink_to_front_, i,
                                   left_offset_v.at(i), right_offset_v.at(i));
  }

  ROS_DEBUG("left: %d, right: %d", (int32_t)left_offset_v.size(), (int32_t)right_offset_v.size());


  // create polygon
  poly_ = createPolygon(ext_lane, left_offset_v, right_offset_v, idx_s, idx_e);


  // point cloud error handling
  if (pc_.data.empty() || points_thr_ == 0)
  {
    ROS_WARN("point cloud is not subscribed or invalid!, points_thr_ = %d, pc.siize = %lu", points_thr_, pc_.data.size());
    return NotReplan();
  }


  // is pointcloud in polygon?
  const auto pcl_pair = findPointCloudInPolygon(poly_, pc_, points_thr_);

  if (!pcl_pair.first)
    return NotReplan();


  // closest pointcloud
  const auto clst_obst = findClosestPointPosAndIdx(ext_lane, left_offset_v, right_offset_v, pcl_pair.second, idx_s, idx_e);
  if(!std::get<0>(clst_obst))
    return NotReplan();

  const auto &clst_idx = std::get<1>(clst_obst);
  const auto &clst_point = std::get<2>(clst_obst);
  ROS_DEBUG("closest point before: %d, (%lf, %lf)", clst_idx, clst_point.x, clst_point.y);

  const auto pnt_s = ext_lane.points.at(clst_idx).pose.position;
  const auto pnt_e = ext_lane.points.at(clst_idx + 1).pose.position;


  // calculate stop_factor_pose (the foot of perpendicular line)
  const auto fop_pair = calcFopPose(pnt_s, pnt_e, clst_point);
  if (!fop_pair.first)
    return NotReplan();

  perpendicular_pose_ = fop_pair.second;

  stop_factor_pose_.position = clst_point;


  // calc actual stop pose
  const auto actual_stop_dist = (direction == 0) ? (stop_distance_ + vehicle_length_baselink_to_front_)
                                           : (stop_distance_ + vehicle_length_baselink_to_rear_);
  const auto asp_tuple = planning_utils::calcDistanceConsideredPoseAndIdx(ext_lane,  perpendicular_pose_, clst_idx, actual_stop_dist, 1);
  if(!std::get<0>(asp_tuple))
    return NotReplan();

  const auto asp_idx = std::get<1>(asp_tuple);
  actual_stop_pose_ = std::get<2>(asp_tuple);


  // wall pose
  const auto wall_tuple = planning_utils::calcDistanceConsideredPoseAndIdx(ext_lane,  perpendicular_pose_, clst_idx, stop_distance_, 1);
  if(!std::get<0>(wall_tuple))
    return NotReplan();

  wall_pose_ = std::get<2>(wall_tuple);


  // replan
  if(asp_idx < (int32_t)out.points.size())
  {
    for(int32_t k = (asp_idx + 1); k < (int32_t)out.points.size(); k++)
    {
      out.points.at(k).twist.linear.x = 0.0;
    }
  }

  autoware_planning_msgs::TrajectoryPoint tp;
  tp.pose = actual_stop_pose_;
  tp.twist.linear.x = 0.0;
  out.points.insert(out.points.begin() + asp_idx + 1, tp);


  return Replan();
}

int ObstacleConsideredLane::getBehindLengthClosest(const autoware_planning_msgs::Trajectory &lane, const int start,
                                                   const double &length)
{
  if (start < 0 || start > (int)lane.points.size() - 1)
  {
    ROS_ERROR("[obstacle_considered_lane] : getBehindLengthClosest() : bad index!");
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
    ROS_ERROR("[obstacle_considered_lane] : getBehindLengthClosest() : bad index");
    return start;
  }
  else
  {
    return idx_min;
  }
}

bool ObstacleConsideredLane::calcVehicleShapeDetectionWidth(const autoware_planning_msgs::Trajectory &lane, const double &shape_tread,
                                                            const double &shape_length, const int idx, double &left_y,
                                                            double &right_y)
{
  left_y = shape_tread * 0.5;
  right_y = shape_tread * 0.5;

  geometry_msgs::Point p_fl_shape;  // front left point in local coordinate
  p_fl_shape.x = shape_length;
  p_fl_shape.y = shape_tread * 0.5;

  geometry_msgs::Point p_fr_shape;  // front right point in local coordinate
  p_fr_shape.x = shape_length;
  p_fr_shape.y = -shape_tread * 0.5;

  const geometry_msgs::Pose base_p = lane.points.at(idx).pose;

  int idx_target = getBehindLengthClosest(lane, idx, shape_length);  // calc waypoint behind baselink_to_front length
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

visualization_msgs::Marker displayObstaclePerpendicularPoint(const geometry_msgs::Pose &pose, int8_t kind)
{
  ROS_DEBUG_STREAM(__func__);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "obstacle_perpendicular_point";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  if (kind == 0 /* no obstacle */)
  {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  else
  {
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 2.0;
    marker.frame_locked = true;
    marker.pose = pose;
    marker.pose.position.z += marker.scale.z / 2;
    marker.color = setColorWhite();
  }
  return marker;
}

visualization_msgs::Marker displayObstaclePoint(const geometry_msgs::Pose &pose, int8_t kind)
{
  ROS_DEBUG_STREAM(__func__);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "obstacle_point";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::SPHERE;
  if (kind == 0 /* no obstacle */)
  {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  else
  {
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.frame_locked = true;
    marker.pose = pose;
    marker.color = setColorWhite();
  }
  return marker;
}

visualization_msgs::MarkerArray displayActiveDetectionArea(const PolygonX &poly, int8_t kind)
{
  ROS_DEBUG_STREAM(__func__);

  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker m_poly;
  m_poly.header.frame_id = "map";
  m_poly.header.stamp = ros::Time();
  m_poly.ns = "active_detection_area";
  m_poly.id = 0;
  m_poly.type = visualization_msgs::Marker::LINE_STRIP;
  m_poly.scale.x = 0.05;
  m_poly.frame_locked = true;

  visualization_msgs::Marker m_point;
  m_point.header.frame_id = "map";
  m_point.header.stamp = ros::Time();
  m_point.ns = "active_detection_area_point";
  m_point.id = 0;
  m_point.type = visualization_msgs::Marker::SPHERE_LIST;
  m_point.scale.x = 0.1;
  m_point.scale.y = 0.1;
  m_point.frame_locked = true;


  if (!poly.empty())  // visualize active polygons
  {
    m_poly.action = visualization_msgs::Marker::ADD;
    m_poly.color = *setColorDependsOnObstacleKind(kind);

    m_point.action = visualization_msgs::Marker::ADD;
    m_point.color = setColorWhite();

    // push back elements
    for (const auto &e : poly)
    {
      m_poly.points.push_back(e);
      m_point.points.push_back(e);
    }

    // insert left first element again to connect
    m_poly.points.push_back(poly.front());
  }
  else
  {
    m_poly.action = visualization_msgs::Marker::DELETE;
    m_point.action = visualization_msgs::Marker::DELETE;
  }

  ma.markers.push_back(m_poly);
  ma.markers.push_back(m_point);

  return ma;
}

std::pair<geometry_msgs::Point, geometry_msgs::Point> calcVehicleFrontPosition(const geometry_msgs::Pose &curr_pose,
                                                                               double base_link_to_front, double width)
{
  geometry_msgs::Pose left;
  left.position.x += base_link_to_front;
  left.position.y += width / 2;

  auto left_abs = planning_utils::transformToAbsoluteCoordinate2D(left.position, curr_pose);

  geometry_msgs::Pose right;
  right.position.x += base_link_to_front;
  right.position.y -= width / 2;

  auto right_abs = planning_utils::transformToAbsoluteCoordinate2D(right.position, curr_pose);

  return std::make_pair(left_abs, right_abs);
}

std::pair<geometry_msgs::Point, geometry_msgs::Point> calcVehicleRearPosition(const geometry_msgs::Pose &curr_pose,
                                                                               double base_link_to_rear, double width)
{
  geometry_msgs::Pose left;
  left.position.x -= base_link_to_rear;
  left.position.y += width / 2;

  auto left_abs = planning_utils::transformToAbsoluteCoordinate2D(left.position, curr_pose);

  geometry_msgs::Pose right;
  right.position.x -= base_link_to_rear;
  right.position.y -= width / 2;

  auto right_abs = planning_utils::transformToAbsoluteCoordinate2D(right.position, curr_pose);

  return std::make_pair(left_abs, right_abs);
}

std::tuple<bool, int32_t, int32_t> calcSearchRange(const autoware_planning_msgs::Trajectory &lane, const geometry_msgs::Pose &curr_pose,
                                                   const std::pair<geometry_msgs::Point, geometry_msgs::Point> &edge_pos,
                                                   double search_distance, int8_t direction)
{
  auto clst_pair =
      planning_utils::findClosestIdxWithDistAngThr(planning_utils::extractPoses(lane), curr_pose, 3.0, M_PI_4);

  if (!clst_pair.first)
  {
    ROS_WARN("Cannot find closest");
    return std::make_tuple(false, -1, -1);
  }

  ROS_DEBUG("clst idx: %d", clst_pair.second);

  // search start and end index
  for (uint32_t i = clst_pair.second; i < lane.points.size(); i++)
  {
    const auto &e = lane.points.at(i).pose;
    const auto &front_left = edge_pos.first;
    const auto &front_right = edge_pos.second;

    auto rel_left = planning_utils::transformToRelativeCoordinate2D(front_left, e);
    auto rel_right = planning_utils::transformToRelativeCoordinate2D(front_right, e);

    ROS_DEBUG("rel_left: %lf, %lf", rel_left.x, rel_left.y);

    ROS_DEBUG("rel_right: %lf, %lf", rel_right.x, rel_right.y);

    if((i == (lane.points.size() - 1))
    || ((direction == 0) && (rel_left.x < 0 || rel_right.x < 0))
    || ((direction == 1) && (rel_left.x > 0 || rel_right.x > 0))) // front
    {
      auto start = i > 0 ? (i - 1) : i;
      return std::make_tuple(true, start, calcForwardIdxByLineIntegral(lane, i, search_distance).second);
    }
  }

  ROS_ERROR("cannot find closest: reach end");
  return std::make_tuple(false, -1, -1);
}

bool insertExtendWaypoint(const autoware_planning_msgs::Trajectory &lane, const double extend_size, autoware_planning_msgs::Trajectory &out)
{
  if (lane.points.empty())
  {
    ROS_ERROR("lane is empty");
    return false;
  }

  geometry_msgs::Pose ext_pose;

  const auto direction = planning_utils::getLaneDirection(planning_utils::extractPoses(lane));
  if(direction == 2)
    return false;

  int32_t sign = [direction](){
    if(direction == 0)
      return 1;
    else
      return -1;
  }();

  geometry_msgs::Point ext_p;
  ext_p.x = sign * extend_size;

  ext_pose.position = planning_utils::transformToAbsoluteCoordinate2D(ext_p, lane.points.back().pose);
  ext_pose.orientation = lane.points.back().pose.orientation;

  out = lane;
  out.points.push_back(out.points.back());
  out.points.back().pose = ext_pose;

  return true;
}

PolygonX createPolygon(const autoware_planning_msgs::Trajectory &lane, const std::vector<double> &left_ofs,
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

std::tuple<bool, int32_t, geometry_msgs::Point> findClosestPointPosAndIdx(const autoware_planning_msgs::Trajectory &lane,
                                                                          const std::vector<double> &left_ofs,
                                                                          const std::vector<double> &right_ofs,
                                                                          const std::vector<geometry_msgs::Point> &points,
                                                                          int32_t idx_s, int32_t idx_e)
{
  ROS_DEBUG_STREAM(__FUNCTION__);
  if(idx_s >= idx_e)
  {
    ROS_ERROR("idx_s: %d is larger than idx_e: %d", idx_s, idx_e);
    return std::make_tuple(false, -1, geometry_msgs::Point());
  }

  int32_t idx_s_loop = idx_s;
  int32_t idx_e_loop = idx_e;
  while(true)
  {
    const int32_t idx_m_loop = (int32_t)((idx_e_loop + idx_s_loop) / 2);
    ROS_DEBUG("idx_s: %d, idx_m: %d, idx_e: %d", idx_s_loop, idx_m_loop, idx_e_loop);

    ROS_DEBUG("create polygon1");
    auto poly1 = createPolygon(lane, left_ofs, right_ofs, idx_s_loop, idx_m_loop);

    // is points in polygon?
    ROS_DEBUG("points_pair1");
    const auto points_pair1 = findPointsInPolygon(poly1, points);

    if(points_pair1.first)
    {
      if ((idx_m_loop - idx_s_loop) == 1)
      {
        ROS_DEBUG("last polygon1");
        auto clst_p = calcClosestPointByXAxis(lane.points.at(idx_s_loop).pose, points_pair1.second);
        ROS_DEBUG("end: idx: %d, clst_p: (%lf, %lf)", idx_s_loop, clst_p.x, clst_p.y);
        return std::make_tuple(true, idx_s_loop, clst_p);
      }

      ROS_DEBUG("select poly1, continue");
      idx_e_loop = idx_m_loop;
      continue;
    }

    ROS_DEBUG("create polygon2");
    auto poly2 = createPolygon(lane, left_ofs, right_ofs, idx_m_loop, idx_e_loop);

    ROS_DEBUG("points_pair2");
    const auto points_pair2 = findPointsInPolygon(poly2, points);
    if(points_pair2.first)
    {
      if ((idx_e_loop - idx_m_loop) == 1)
      {
        ROS_DEBUG("last polygon2");
        auto clst_p = calcClosestPointByXAxis(lane.points.at(idx_m_loop).pose, points_pair2.second);
        ROS_DEBUG("end: idx: %d, clst_p: (%lf, %lf)", idx_m_loop, clst_p.x, clst_p.y);
        return std::make_tuple(true, idx_m_loop, clst_p);
      }

      ROS_DEBUG("select poly2, continue");
      idx_s_loop = idx_m_loop;
      continue;
    }


    ROS_ERROR("nothing is polygon");
    return std::make_tuple(false, -1, geometry_msgs::Point());
  }
}

std::pair<bool, geometry_msgs::Pose> calcFopPose(const geometry_msgs::Point &line_s, const geometry_msgs::Point &line_e,
                                                 geometry_msgs::Point point)
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
}
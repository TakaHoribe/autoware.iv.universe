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

// ROS includes

// User defined includes
#include "stop_planner/stop_factor_search_utils.h"
#include <sensor_msgs/point_cloud2_iterator.h>

namespace motion_planner
{

std::vector<geometry_msgs::Point> createLattice(const geometry_msgs::Pose &pose, double height, double width,
                                                double count)
{
  ROS_DEBUG_STREAM(__func__);
  std::vector<geometry_msgs::Point> points;

  points.push_back(pose.position);
  geometry_msgs::Point p_top = pose.position;
  p_top.z += height;
  points.push_back(p_top);

  for (int32_t i = 1; i < ((int32_t)(count / 2) + 1); i++)
  {
    geometry_msgs::Point p_bottom_l;
    p_bottom_l.y += width / count * i;
    geometry_msgs::Point trans_pbl = planning_utils::transformToAbsoluteCoordinate2D(p_bottom_l, pose);
    points.push_back(trans_pbl);

    geometry_msgs::Point p_top_l;
    p_top_l.y += width / count * i;
    geometry_msgs::Point trans_ptl = planning_utils::transformToAbsoluteCoordinate2D(p_top_l, pose);
    trans_ptl.z += height;
    points.push_back(trans_ptl);

    geometry_msgs::Point p_bottom_r;
    p_bottom_r.y -= width / count * i;
    geometry_msgs::Point trans_pbr = planning_utils::transformToAbsoluteCoordinate2D(p_bottom_r, pose);
    points.push_back(trans_pbr);

    geometry_msgs::Point p_top_r;
    p_top_r.y -= width / count * i;
    geometry_msgs::Point trans_ptr = planning_utils::transformToAbsoluteCoordinate2D(p_top_r, pose);
    trans_ptr.z += height;
    points.push_back(trans_ptr);

    if (i == (count / 2))
    {
      points.push_back(trans_pbl);
      points.push_back(trans_pbr);
      points.push_back(trans_ptl);
      points.push_back(trans_ptr);
    }
  }
  return points;
}

std_msgs::ColorRGBA setColorWhite()
{
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 1.0;

  return color;
}

std_msgs::ColorRGBA setColorGray()
{
  std_msgs::ColorRGBA color;
  color.r = 0.5;
  color.g = 0.5;
  color.b = 0.5;
  color.a = 1.0;

  return color;
}

std_msgs::ColorRGBA setColorYellow()
{
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 1.0;

  return color;
}


std::unique_ptr<std_msgs::ColorRGBA> setColorDependsOnObstacleKind(int8_t kind)
{
  std::unique_ptr<std_msgs::ColorRGBA> color(new std_msgs::ColorRGBA);

  color->a = 1.0;
  if (kind == 0)
  {
    color->r = 1.0;
    color->g = 0.0;
    color->b = 1.0;
  }
  else if (kind == 1)
  {
    color->r = 0.0;
    color->g = 0.0;
    color->b = 1.0;
  }
  else if (kind == 2)
  {
    color->r = 1.0;
    color->g = 0.0;
    color->b = 0.0;
  }
  else if (kind == 3)
  {
    color->r = 1.0;
    color->g = 1.0;
    color->b = 0.0;
  }
  else if (kind == 4)
  {
    color->r = 0.0;
    color->g = 1.0;
    color->b = 1.0;
  }
  else if (kind == 5)
  {
    color->r = 0.0;
    color->g = 1.0;
    color->b = 0.0;
  }
  else
  {
    color->r = 1.0;
    color->g = 1.0;
    color->b = 1.0;
  }

  return color;
}

// display the next waypoint by markers.
visualization_msgs::Marker displayWall(const geometry_msgs::Pose &pose, int8_t kind, int32_t id)
{
  ROS_DEBUG_STREAM(__func__);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.id = id;
  marker.ns = "stop_factor_wall";
  marker.type = visualization_msgs::Marker::LINE_LIST;
  if (kind == 0)
  {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  else
  {
    marker.action = visualization_msgs::Marker::ADD;
    marker.points = createLattice(pose, 2.0, 8.0, 6);

  }
  marker.lifetime = ros::Duration(10.0);
  marker.scale.x = 0.1;
  marker.frame_locked = true;
  marker.color = *setColorDependsOnObstacleKind(kind);
  
  return marker;
}


std::pair<bool, int32_t> calcForwardIdxByLineIntegral(const autoware_planning_msgs::Trajectory &lane, int32_t base_idx, double stop_offset_dist)
{
  ROS_DEBUG_STREAM(__func__);

  if(lane.points.empty() || (base_idx < 0) || base_idx > (int32_t)(lane.points.size() - 1))
    return std::make_pair(false, -1);

  int32_t actual_idx = base_idx;
  double accum = 0.0;
  while(actual_idx < (int32_t)lane.points.size())
  {
    if(actual_idx == (int32_t)lane.points.size() -1)
    {
      ROS_DEBUG("idx: %d, accum: %lf, dist: %lf", actual_idx, accum, stop_offset_dist);
      return std::make_pair(true, actual_idx);
    }

    accum += planning_utils::calcDistance2D(lane.points.at(actual_idx + 1).pose.position,
                                            lane.points.at(actual_idx).pose.position);

    if(accum > stop_offset_dist)
    {
      ROS_DEBUG("idx: %d, accum: %lf, dist: %lf", actual_idx, accum, stop_offset_dist);
      return std::make_pair(true, actual_idx + 1);
    }

    actual_idx++;
  }

  return std::make_pair(false, -1);
}

std::pair<bool, std::vector<geometry_msgs::Point>> findPointCloudInPolygon(const PolygonX &poly, const sensor_msgs::PointCloud2 &pc, int32_t points_thr)
{
  std::vector<geometry_msgs::Point> pc_vec;
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
      pc_vec.push_back(p);
    }
  }
  ROS_DEBUG("detected, p_count: %d", (int32_t)pc_vec.size());

  if ((int32_t)pc_vec.size() >= points_thr)
  {
    return std::make_pair(true, pc_vec);
  }
  else
  {
    return std::make_pair(false, std::vector<geometry_msgs::Point>());
  }
}


std::pair<bool, std::vector<geometry_msgs::Point>> findPointsInPolygon(const PolygonX &poly, const std::vector<geometry_msgs::Point> &points,
    int32_t points_thr)
{
  std::vector<geometry_msgs::Point> pc_vec;
  for (const auto &p : points)
  {
    if (planning_utils::isInPolygon(poly, p))
    {
      pc_vec.push_back(p);
    }
  }
  ROS_DEBUG("detected, p_count: %d", (int32_t)pc_vec.size());

  if ((int32_t)pc_vec.size() >= points_thr)
  {
    return std::make_pair(true, pc_vec);
  }
  else
  {
    return std::make_pair(false, std::vector<geometry_msgs::Point>());
  }
}

geometry_msgs::Point calcClosestPointByXAxis(const geometry_msgs::Pose &pose, const std::vector<geometry_msgs::Point> &points)
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


}  // namespace motion_planner
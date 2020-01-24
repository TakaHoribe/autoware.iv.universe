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

// User defined includes
#include "stop_planner/visualize.h"
#include <sensor_msgs/point_cloud2_iterator.h>

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
  marker.lifetime = ros::Duration(0.5);
  marker.scale.x = 0.1;
  marker.frame_locked = true;
  marker.color = *setColorDependsOnObstacleKind(kind);
  
  return marker;
}


visualization_msgs::Marker displayObstaclePerpendicularPoint(const geometry_msgs::Pose &pose, int8_t kind)
{
  ROS_DEBUG_STREAM(__func__);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "obstacle_perpendicular_point";
  marker.id = 0;
  marker.lifetime = ros::Duration(0.5);
  marker.type = visualization_msgs::Marker::CUBE;
  if (kind == 0 /* no obstacle */)
  {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  else
  {
    marker.action = visualization_msgs::Marker::ADD;
  }
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 2.0;
  marker.frame_locked = true;
  marker.pose = pose;
  marker.pose.position.z += marker.scale.z / 2;
  marker.color = setColorWhite();
  
  return marker;
}

visualization_msgs::Marker displayObstaclePoint(const geometry_msgs::Pose &pose, int8_t kind)
{
  ROS_DEBUG_STREAM(__func__);
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();
  marker.ns = "obstacle_point";
  marker.id = 0;
  marker.lifetime = ros::Duration(0.5);
  marker.type = visualization_msgs::Marker::SPHERE;
  if (kind == 0 /* no obstacle */)
  {
    marker.action = visualization_msgs::Marker::DELETE;
  }
  else
  {
    marker.action = visualization_msgs::Marker::ADD;
  }
  marker.scale.x = 0.3;
  marker.scale.y = 0.3;
  marker.scale.z = 0.3;
  marker.frame_locked = true;
  marker.pose = pose;
  marker.color = setColorWhite();

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
  m_poly.lifetime = ros::Duration(0.5);

  visualization_msgs::Marker m_point;
  m_point.header.frame_id = "map";
  m_point.header.stamp = ros::Time();
  m_point.ns = "active_detection_area_point";
  m_point.id = 0;
  m_point.type = visualization_msgs::Marker::SPHERE_LIST;
  m_point.scale.x = 0.1;
  m_point.scale.y = 0.1;
  m_point.frame_locked = true;
  m_point.lifetime = ros::Duration(0.5);


  if (!poly.empty())  // visualize active polygons
  {
    m_poly.action = visualization_msgs::Marker::ADD;
    m_poly.color = *setColorDependsOnObstacleKind(kind);

    m_point.action = visualization_msgs::Marker::ADD;
    m_point.color = *setColorDependsOnObstacleKind(kind);

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



std_msgs::ColorRGBA setColorWhite()
{
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 1.0;
  color.a = 0.999;

  return color;
}

std_msgs::ColorRGBA setColorGray()
{
  std_msgs::ColorRGBA color;
  color.r = 0.5;
  color.g = 0.5;
  color.b = 0.5;
  color.a = 0.999;

  return color;
}

std_msgs::ColorRGBA setColorYellow()
{
  std_msgs::ColorRGBA color;
  color.r = 1.0;
  color.g = 1.0;
  color.b = 0.0;
  color.a = 0.999;

  return color;
}


std::unique_ptr<std_msgs::ColorRGBA> setColorDependsOnObstacleKind(int8_t kind)
{
  std::unique_ptr<std_msgs::ColorRGBA> color(new std_msgs::ColorRGBA);

  color->a = 0.999;
  if (kind == 0)
  {
    color->r = 0.0;
    color->g = 1.0;
    color->b = 1.0;
  }
  else if (kind == 1)
  {
    color->r = 1.0;
    color->g = 0.0;
    color->b = 0.0;
  }
  else if (kind == 2)
  {
    color->r = 0.0;
    color->g = 0.0;
    color->b = 1.0;
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

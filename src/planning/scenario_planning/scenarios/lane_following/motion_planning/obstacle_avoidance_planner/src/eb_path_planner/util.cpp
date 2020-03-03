/*
 * Copyright 2018 Autoware Foundation. All rights reserved.
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
 *
 * v1.0 Yukihiro Saito
 */

#include <geometry_msgs/Pose.h>
#include <nav_msgs/MapMetaData.h>

#include <tf2/utils.h>

#include <Eigen/Core>

#include "eb_path_planner/util.h"
#include "eb_path_planner/spline_interpolate.h"

namespace util
{
  // ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): retative, (px, py): absolute, (ox, oy): origin
// (pu, pv) = rot^-1 * {(px, py) - (ox, oy)}
geometry_msgs::Point transformToRelativeCoordinate2D(
  const geometry_msgs::Point &point,
  const geometry_msgs::Pose &origin)
{
  // translation
  geometry_msgs::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::Point res;
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}

double calculateEigen2DDistance(const Eigen::Vector2d& a, 
                                const Eigen::Vector2d& b)
{
  double dx = a(0)-b(0);
  double dy = a(1)-b(1);
  return std::sqrt(dx*dx+dy*dy);
}

bool transformMapToImage(const geometry_msgs::Point& map_point,
                         const nav_msgs::MapMetaData& occupancy_grid_info,
                         geometry_msgs::Point& image_point)
{
  geometry_msgs::Point relative_p = 
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = relative_p.x/resolution;
  double map_y_in_image_resolution = relative_p.y/resolution;
  double image_x = map_y_height - map_y_in_image_resolution;
  double image_y = map_x_width - map_x_in_image_resolution;
  if(image_x>=0 && 
     image_x<(int)map_y_height &&
     image_y>=0 && 
     image_y<(int)map_x_width)
  {
    image_point.x = image_x;
    image_point.y = image_y;
    return true;
  }
  else
  {
    return false;
  } 
}


bool transformImageToMap(const geometry_msgs::Point& image_point,
                         const nav_msgs::MapMetaData& occupancy_grid_info,
                         geometry_msgs::Point& map_point)
{
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = map_x_width - image_point.y;
  double map_y_in_image_resolution = map_y_height - image_point.x;
  double relative_x = map_x_in_image_resolution*resolution;
  double relative_y = map_y_in_image_resolution*resolution;
  double yaw = tf2::getYaw(occupancy_grid_info.origin.orientation);
  geometry_msgs::Point res;
  res.x = (cos(-yaw) * relative_x) + (sin(-yaw) * relative_y);
  res.y = ((-1) * sin(-yaw) * relative_x) + (cos(-yaw) * relative_y);
  
  map_point.x = res.x + occupancy_grid_info.origin.position.x;
  map_point.y = res.y + occupancy_grid_info.origin.position.y;
  map_point.z = occupancy_grid_info.origin.position.z;
  return true;
}

bool interpolate2DPoints(const std::vector<double>& base_x, 
                         const std::vector<double>& base_y,
                         const double resolution,
                         std::vector<geometry_msgs::Point>& interpolated_points)
{
  if(base_x.empty()||base_y.empty())
  {
    return false;
  }
  std::vector<double> base_s = spline::calcEuclidDist(base_x, base_y);
  if(base_s.empty())
  {
    return false;
  }
  std::vector<double> new_s;
  for(double i = 0.0; 
      i <= base_s.back();
      i += resolution)
  {
    new_s.push_back(i);
  }
  new_s.push_back(base_s.back());
  spline::SplineInterpolate spline;
  std::vector<double> interpolated_x;
  std::vector<double> interpolated_y;
  spline.interpolate(base_s, base_x, new_s, interpolated_x);
  spline.interpolate(base_s, base_y, new_s, interpolated_y);
  for (size_t i = 0; i < interpolated_x.size(); i++)
  {
    geometry_msgs::Point point;
    point.x = interpolated_x[i];
    point.y = interpolated_y[i];
    interpolated_points.push_back(point);
  }
}
}
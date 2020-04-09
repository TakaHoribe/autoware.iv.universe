/*
 * Copyright 2020 Tier IV, Inc. All rights reserved.
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
#ifndef UTIL__H
#define UTIL__H

#include <Eigen/Core>

namespace util {
geometry_msgs::Point transformToRelativeCoordinate2D(const geometry_msgs::Point& point,
                                                     const geometry_msgs::Pose& origin);

double calculateEigen2DDistance(const Eigen::Vector2d& a, const Eigen::Vector2d& b);

bool transformMapToImage(const geometry_msgs::Point& map_point, const nav_msgs::MapMetaData& occupancy_grid_info,
                         geometry_msgs::Point& image_point);

bool transformImageToMap(const geometry_msgs::Point& image_point, const nav_msgs::MapMetaData& occupancy_grid_info,
                         geometry_msgs::Point& map_point);

bool interpolate2DPoints(const std::vector<double>& x, const std::vector<double>& y, const double resolution,
                         std::vector<geometry_msgs::Point>& interpolated_points);
}  // namespace util

#endif
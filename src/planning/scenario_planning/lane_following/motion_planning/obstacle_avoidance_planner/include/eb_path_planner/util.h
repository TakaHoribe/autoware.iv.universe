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
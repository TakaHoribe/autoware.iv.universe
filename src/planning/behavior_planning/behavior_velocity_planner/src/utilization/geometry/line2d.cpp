#include <utilization/geometry/line2d.hpp>
#include <iostream>

Line2d::Line2d() {}
Line2d::Line2d(const Eigen::Vector2d &point1, const Eigen::Vector2d &point2)
    : vec1_(point1), vec2_(point2) {}

bool Line2d::getBackwordPointFromBasePoint(const Eigen::Vector2d &line_point1,
                                           const Eigen::Vector2d &line_point2,
                                           const Eigen::Vector2d &base_point,
                                           const double backward_length,
                                           Eigen::Vector2d &output_point)
{
    Eigen::Vector2d line_vec = line_point2 - line_point1;
    Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
    output_point = base_point + backward_vec;
    return true;
}
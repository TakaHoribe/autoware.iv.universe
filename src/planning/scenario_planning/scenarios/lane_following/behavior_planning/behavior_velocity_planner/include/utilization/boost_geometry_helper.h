#pragma once

#include <vector>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/register/point.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/PathPointWithLaneId.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>

using Point2d = boost::geometry::model::d2::point_xy<double>;
using LineString2d = boost::geometry::model::linestring<Point2d>;
using Polygon2d = boost::geometry::model::polygon<Point2d, false>;

BOOST_GEOMETRY_REGISTER_POINT_3D(geometry_msgs::Point, double, cs::cartesian, x, y, z)
BOOST_GEOMETRY_REGISTER_POINT_3D(geometry_msgs::PoseWithCovarianceStamped, double, cs::cartesian, pose.pose.position.x,
                                 pose.pose.position.y, pose.pose.position.z)

BOOST_GEOMETRY_REGISTER_POINT_3D(autoware_planning_msgs::PathPoint, double, cs::cartesian, pose.position.x,
                                 pose.position.y, pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(autoware_planning_msgs::PathPointWithLaneId, double, cs::cartesian,
                                 point.pose.position.x, point.pose.position.y, point.pose.position.z)
BOOST_GEOMETRY_REGISTER_POINT_3D(autoware_planning_msgs::TrajectoryPoint, double, cs::cartesian, pose.position.x,
                                 pose.position.y, pose.position.z)

template <class T>
Point2d to_bg2d(const T& p) {
  return Point2d(boost::geometry::get<0>(p), boost::geometry::get<1>(p));
}

template <class T>
LineString2d to_bg2d(const std::vector<T>& vec) {
  LineString2d ps;
  for (const auto& p : vec) {
    ps.push_back(to_bg2d(p));
  }
  return ps;
}

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

#include "stop_planner/planning_utils.h"
#include "stop_planner/visualize.h"
#include "stop_planner/interpolate.h"

namespace planning_utils
{


double calcDistance2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q)
{
  const double dx = p.x - q.x;
  const double dy = p.y - q.y;
  return sqrt(dx * dx + dy * dy);
}

double calcDistSquared2D(const geometry_msgs::Point &p, const geometry_msgs::Point &q)
{
  const double dx = p.x - q.x;
  const double dy = p.y - q.y;
  return (dx * dx + dy * dy);
}


/* a_vec = line_e - line_s, b_vec = point - line_s
 * a_vec x b_vec = |a_vec| * |b_vec| * sin(theta)
 *               = |a_vec| * lateral_error ( because, lateral_error = |b_vec| * sin(theta) )
 *
 * lateral_error = a_vec x b_vec / |a_vec|
 *        = (a_x * b_y - a_y * b_x) / |a_vec| */
double calcLateralError2D(const geometry_msgs::Point &line_s, const geometry_msgs::Point &line_e,
                          const geometry_msgs::Point &point)
{
  tf2::Vector3 a_vec((line_e.x - line_s.x), (line_e.y - line_s.y), 0.0);
  tf2::Vector3 b_vec((point.x - line_s.x), (point.y - line_s.y), 0.0);

  double lat_err = (a_vec.length() > 0) ? a_vec.cross(b_vec).z() / a_vec.length() : 0.0;
  return lat_err;
}


// get closest point index from current pose
bool findClosestIdxWithDistAngThr(const autoware_planning_msgs::Trajectory &in_trajectory,
                                                      const geometry_msgs::Pose &curr_pose, int32_t &out_idx, double dist_thr, double angle_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  out_idx = -1;

  for (int32_t i = 0; i < (int32_t)in_trajectory.points.size(); ++i)
  {
    const double ds = calcDistSquared2D(in_trajectory.points.at(i).pose.position, curr_pose.position);
    if (ds > dist_thr * dist_thr)
      continue;

    double yaw_pose = tf2::getYaw(curr_pose.orientation);
    double yaw_ps = tf2::getYaw(in_trajectory.points.at(i).pose.orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_ps);
    if (fabs(yaw_diff) > angle_thr)
      continue;

    if (ds < dist_squared_min)
    {
      dist_squared_min = ds;
      out_idx = i;
    }
  }

  return (out_idx >= 0) ? true : false;
}

template <typename T>
bool isInPolygon(const std::vector<T> &polygon, const T &point)
{
  // polygons with fewer than 3 sides are excluded
  if(polygon.size() < 3)
    return false;

  bool in_poly = false;
  double x1, x2, y1, y2;

  uint32_t nr_poly_points = polygon.size();
  // start with the last point to make the check last point<->first point the first one
  double xold = polygon.at(nr_poly_points - 1).x();
  double yold = polygon.at(nr_poly_points - 1).y();
  for (const auto &poly_p : polygon)
  {
    double xnew = poly_p.x();
    double ynew = poly_p.y();
    if (xnew > xold)
    {
      x1 = xold;
      x2 = xnew;
      y1 = yold;
      y2 = ynew;
    }
    else
    {
      x1 = xnew;
      x2 = xold;
      y1 = ynew;
      y2 = yold;
    }

    if ((xnew < point.x()) == (point.x() <= xold) && (point.y() - y1) * (x2 - x1) < (y2 - y1) * (point.x() - x1))
    {
      in_poly = !in_poly;
    }
    xold = xnew;
    yold = ynew;
  }

  return (in_poly);
}

template <>
bool isInPolygon(const std::vector<geometry_msgs::Point> &polygon, const geometry_msgs::Point &point)
{
  std::vector<tf2::Vector3> polygon_conv;
  for(const auto &el : polygon)
    polygon_conv.emplace_back(el.x, el.y, el.z);

  tf2::Vector3 point_conv = tf2::Vector3(point.x, point.y, point.z);

  return isInPolygon<tf2::Vector3>(polygon_conv, point_conv);
}

double normalizeEulerAngle(double euler)
{
  double res = euler;
  while (res > M_PI)
  {
    res -= (2 * M_PI);
  }
  while (res < -M_PI)
  {
    res += 2 * M_PI;
  }

  return res;
}

// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): retative, (px, py): absolute, (ox, oy): origin
// (px, py) = rot * (pu, pv) + (ox, oy)
geometry_msgs::Point transformToAbsoluteCoordinate2D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &origin)
{
  // rotation
  geometry_msgs::Point rot_p;
  double yaw = tf2::getYaw(origin.orientation);
  rot_p.x = (cos(yaw) * point.x) + ((-1) * sin(yaw) * point.y);
  rot_p.y = (sin(yaw) * point.x) + (cos(yaw) * point.y);

  // translation
  geometry_msgs::Point res;
  res.x = rot_p.x + origin.position.x;
  res.y = rot_p.y + origin.position.y;
  res.z = origin.position.z;

  return res;
}

geometry_msgs::Point transformToAbsoluteCoordinate3D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &origin)
{
  Eigen::Translation3d trans(origin.position.x, origin.position.y, origin.position.z);
  Eigen::Quaterniond rot(origin.orientation.w, origin.orientation.x, origin.orientation.y, origin.orientation.z);

  Eigen::Vector3d v(point.x, point.y, point.z);
  Eigen::Vector3d transformed_v;
  transformed_v = trans * rot.inverse() * v;

  geometry_msgs::Point transformed_p = tf2::toMsg(transformed_v);
  return transformed_p;
}

// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): retative, (px, py): absolute, (ox, oy): origin
// (pu, pv) = rot^-1 * {(px, py) - (ox, oy)}
geometry_msgs::Point transformToRelativeCoordinate2D(const geometry_msgs::Point &point,
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

geometry_msgs::Point transformToRelativeCoordinate3D(const geometry_msgs::Point &point,
                                                                      const geometry_msgs::Pose &origin)
{
  Eigen::Translation3d trans(-origin.position.x, -origin.position.y, -origin.position.z);
  Eigen::Quaterniond rot(origin.orientation.w, origin.orientation.x, origin.orientation.y, origin.orientation.z);

  Eigen::Vector3d v(point.x, point.y, point.z);
  Eigen::Vector3d transformed_v;
  transformed_v = trans * rot * v;

  geometry_msgs::Point transformed_p = tf2::toMsg(transformed_v);
  return transformed_p;
}

geometry_msgs::Quaternion getQuaternionFromYaw(const double &_yaw)
{
  tf2::Quaternion q;
  q.setRPY(0, 0, _yaw);
  return tf2::toMsg(q);
}

void convertEulerAngleToMonotonic(std::vector<double> &a)
{
  for (unsigned int i = 1; i < a.size(); ++i)
  {
    const double da = a[i] - a[i - 1];
    a[i] = a[i - 1] + normalizeEulerAngle(da);
  }
}

bool linearInterpTrajectory(const std::vector<double> &base_index, const autoware_planning_msgs::Trajectory &base_trajectory,
                              const std::vector<double> &out_index, autoware_planning_msgs::Trajectory &out_trajectory)
{
  std::vector<double> px, py, pz, pyaw, tlx, taz, alx, aaz;
  for (const auto &p : base_trajectory.points)
  {
    px.push_back(p.pose.position.x);
    py.push_back(p.pose.position.y);
    pz.push_back(p.pose.position.z);
    pyaw.push_back(tf2::getYaw(p.pose.orientation));
    tlx.push_back(p.twist.linear.x);
    taz.push_back(p.twist.angular.z);
    alx.push_back(p.accel.linear.x);
    aaz.push_back(p.accel.angular.z);
  }

  convertEulerAngleToMonotonic(pyaw);

  std::vector<double> px_p, py_p, pz_p, pyaw_p, tlx_p, taz_p, alx_p, aaz_p;

  if (!LinearInterpolate::interpolate(base_index, px, out_index, px_p) ||
      !LinearInterpolate::interpolate(base_index, py, out_index, py_p) ||
      !LinearInterpolate::interpolate(base_index, pz, out_index, pz_p) ||
      !LinearInterpolate::interpolate(base_index, pyaw, out_index, pyaw_p) ||
      !LinearInterpolate::interpolate(base_index, tlx, out_index, tlx_p) ||
      !LinearInterpolate::interpolate(base_index, taz, out_index, taz_p) ||
      !LinearInterpolate::interpolate(base_index, alx, out_index, alx_p) ||
      !LinearInterpolate::interpolate(base_index, aaz, out_index, aaz_p))
  {
    ROS_WARN("[linearInterpTrajectory] interpolation error!!");
    return false;
  }

  out_trajectory.header = base_trajectory.header;
  out_trajectory.points.clear();
  autoware_planning_msgs::TrajectoryPoint point;
  for (unsigned int i = 0; i < out_index.size(); ++i)
  {
    point.pose.position.x = px_p.at(i);
    point.pose.position.y = py_p.at(i);
    point.pose.position.z = pz_p.at(i);
    point.pose.orientation = getQuaternionFromYaw(pyaw_p.at(i));
    point.twist.linear.x = tlx_p.at(i);
    point.twist.angular.z = taz_p.at(i);
    point.accel.linear.x = alx_p.at(i);
    point.accel.angular.z = aaz_p.at(i);
    out_trajectory.points.push_back(point);
  }
  return true;

}

int calcForwardIdxByLineIntegral(const autoware_planning_msgs::Trajectory &in_trajectory, int32_t start_idx, double distance)

{
  double dist_sum = 0.0;
  for (int i = start_idx; i < (int)in_trajectory.points.size() - 1; ++i)
  {
    dist_sum += planning_utils::calcDistance2D(in_trajectory.points.at(i + 1).pose.position,
                                               in_trajectory.points.at(i).pose.position);
    if (dist_sum > distance)
    {
      return i;
    }
  }
  return (int)(in_trajectory.points.size()) - 1;
}


geometry_msgs::Pose getPoseOnTrajectoryWithRadius(const autoware_planning_msgs::Trajectory &in_trajectory, const geometry_msgs::Point &origin,
                                                  const int start_idx, const double radius)
{
  if (start_idx < 0 || start_idx >= in_trajectory.points.size())
  {
    return geometry_msgs::Pose();
  }

  double prev_dist = 0.0;
  geometry_msgs::Point p_prev = origin;
  if (radius > 0)
  {
    for (uint i = start_idx; i < in_trajectory.points.size() - 1; ++i)
    {
      geometry_msgs::Point p_i = in_trajectory.points.at(i).pose.position;
      double dist_i = planning_utils::calcDistance2D(p_i, origin);
      if (dist_i > radius)
      {
        // points found. do interpolation.
        double d_next = dist_i - radius;
        double d_prev = radius - prev_dist;
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
  else if (radius < 0)
  {
    double abs_radius = std::fabs(radius);
    for (uint i = start_idx; i > 0; --i)
    {
      geometry_msgs::Point p_i = in_trajectory.points.at(i).pose.position;
      double dist_i = planning_utils::calcDistance2D(p_i, origin);
      if (dist_i > abs_radius)
      {
        // points found. do interpolation.
        double d_next = dist_i - abs_radius;
        double d_prev = abs_radius - prev_dist;
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

}  // namespace planning_utils

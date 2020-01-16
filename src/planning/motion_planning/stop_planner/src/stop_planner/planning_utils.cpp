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


std::vector<geometry_msgs::Pose> extractPoses(const autoware_planning_msgs::Trajectory &lane)
{
  std::vector<geometry_msgs::Pose> poses;

  for (const auto &p : lane.points)
    poses.push_back(p.pose);

  return poses;
}

// get closest point index from current pose
std::pair<bool, int32_t> findClosestIdxWithDistAngThr(const std::vector<geometry_msgs::Pose> &curr_ps,
                                                      const geometry_msgs::Pose &curr_pose, double dist_thr, double angle_thr)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  int32_t idx_min = -1;

  for (int32_t i = 0; i < (int32_t)curr_ps.size(); ++i)
  {
    const double ds = calcDistSquared2D(curr_ps.at(i).position, curr_pose.position);
    if (ds > dist_thr * dist_thr)
      continue;

    double yaw_pose = tf2::getYaw(curr_pose.orientation);
    double yaw_ps = tf2::getYaw(curr_ps.at(i).orientation);
    double yaw_diff = normalizeEulerAngle(yaw_pose - yaw_ps);
    if (fabs(yaw_diff) > angle_thr)
      continue;

    if (ds < dist_squared_min)
    {
      dist_squared_min = ds;
      idx_min = i;
    }
  }

  return (idx_min >= 0) ? std::make_pair(true, idx_min) : std::make_pair(false, idx_min);
}

int8_t getLaneDirection(const std::vector<geometry_msgs::Pose> &poses, double dist_thr)
{
  if(poses.size() < 2)
  {
    ROS_ERROR("size of points is smaller than 2");
    return 2;
  }

  for(uint32_t i = 0; i < poses.size(); i++)
  {
    geometry_msgs::Pose prev;
    geometry_msgs::Pose next;

    if(i == (poses.size() - 1))
    {
      prev = poses.at(i - 1);
      next = poses.at(i);
    }
    else
    {
      prev = poses.at(i);
      next = poses.at(i + 1);
    };

    if(planning_utils::calcDistSquared2D(prev.position, next.position) > dist_thr * dist_thr)
    {
      const auto rel_p = transformToRelativeCoordinate2D(next.position, prev);
      return (rel_p.x > 0.0) ? 0 : 1;
    }
  }


  ROS_ERROR("lane is something wrong. poses.size = %lu, dist_thr = %f", poses.size(), dist_thr);
  return 2;

}

bool isDirectionForward(const geometry_msgs::Pose &prev, const geometry_msgs::Pose &next)
{
  return (transformToRelativeCoordinate2D(next.position, prev).x > 0.0) ? true : false;
}

bool isDirectionForward(const geometry_msgs::Pose &prev, const geometry_msgs::Point &next)
{
  return transformToRelativeCoordinate2D(next, prev).x > 0.0;
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

double kmph2mps(double velocity_kmph)
{
  return (velocity_kmph * 1000) / (60 * 60);
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

std::pair<bool, geometry_msgs::Point> calcFootOfPerpendicular(const geometry_msgs::Point &line_s, const geometry_msgs::Point &line_e, const geometry_msgs::Point &point)
{
  constexpr double ER = 1e-5;  // 0.00001

  Eigen::Vector3d vec_a((line_e.x - line_s.x),
                        (line_e.y - line_s.y),
                        (line_e.z - line_s.z));

  if (vec_a.norm() < ER)
    return std::make_pair(false, geometry_msgs::Point());

  double lateral_error = calcLateralError2D(line_s, line_e, point);

  /* calculate the position of the foot of a perpendicular line */
  Eigen::Vector2d uva2d(vec_a.x(), vec_a.y());
  uva2d.normalize();
  Eigen::Rotation2Dd rot = (lateral_error > 0) ? Eigen::Rotation2Dd(-M_PI / 2.0) : Eigen::Rotation2Dd(M_PI / 2.0);
  Eigen::Vector2d uva2d_rot = rot * uva2d;

  geometry_msgs::Point h;
  h.x = point.x + fabs(lateral_error) * uva2d_rot.x();
  h.y = point.y + fabs(lateral_error) * uva2d_rot.y();
  h.z = (line_s.z + line_e.z) / 2;

  return std::make_pair(true, h);
}

std::pair<bool, geometry_msgs::Point> findIntersectionWithLineCircle(const geometry_msgs::Point &line_s,
                                                                     const geometry_msgs::Point &line_e,
                                                                     const geometry_msgs::Point &point,
                                                                     double range,
                                                                     int8_t which_point)
{
  if(range < 0 || abs(which_point) > 1)
    return std::make_pair(false, geometry_msgs::Point());

  constexpr double ER = 1e-5;  // 0.00001
  Eigen::Vector3d vec_a((line_e.x - line_s.x),
                        (line_e.y - line_s.y),
                        (line_e.z - line_s.z));

  if (vec_a.norm() < ER)
    return std::make_pair(false, geometry_msgs::Point());

  double lateral_error = calcLateralError2D(
      line_s, line_e, point);

  if (fabs(lateral_error) > range)
  {
    ROS_ERROR("lateral_error is larger than range, lat_err: %lf, radius: %lf", lateral_error, range);
    return std::make_pair(false, geometry_msgs::Point());
  }

  /* calculate the position of the foot of a perpendicular line */
  auto h = calcFootOfPerpendicular(line_s, line_e, point);

  if(!h.first)
  {
    ROS_ERROR("cannot calc fop");
    return std::make_pair(false, geometry_msgs::Point());
  }

  // if there is a intersection
  if (fabs(fabs(lateral_error) - range) < ER)
  {
    return std::make_pair(true, h.second);
  }
  else
  {
    // if there are two intersection
    // get intersection in front of vehicle
    Eigen::Vector2d uva2d(vec_a.x(), vec_a.y());
    uva2d.normalize();

    double s = sqrt(pow(range, 2) - pow(lateral_error, 2));
    geometry_msgs::Point res;

    if(which_point == 0)
    {
      res.x = h.second.x + s * uva2d.x();
      res.y = h.second.y + s * uva2d.y();
      res.z = (line_s.z + line_e.z) / 2;
    }
    else
    {
      res.x = h.second.x - s * uva2d.x();
      res.y = h.second.y - s * uva2d.y();
      res.z = (line_s.z + line_e.z) / 2;
    }

    return std::make_pair(true, res);
  }
}

std::pair<bool, int32_t> findFirstIdxOutOfRange(const std::vector<geometry_msgs::Pose> &pose_v,
                                                const geometry_msgs::Point &point,
                                                double range, int32_t search_idx_s, int8_t which_dir)
{
  if (pose_v.empty() || search_idx_s < 0 || abs(which_dir) > 1 || range < 0)
    return std::make_pair(false, -1);

  int32_t i = search_idx_s;
  while (true)
  {
    // if search waypoint is the last
    if ((which_dir == 0 && i == (int32_t)pose_v.size() - 1) || (which_dir == 1 && i == 0))
    {
      return std::make_pair(true, i);
    }

    // if there exists an effective waypoint
    const double ds = calcDistSquared2D(point, pose_v.at(i).position);
    if (ds > std::pow(range, 2))
      return std::make_pair(true, i);

    if (which_dir == 0)
      i++;
    else
      i--;
  }
}

std::tuple<bool, int32_t, geometry_msgs::Pose> calcDistanceConsideredPoseAndIdx(const autoware_planning_msgs::Trajectory &lane, const geometry_msgs::Pose &pose,
                                                                                int32_t idx, double stop_dist, int8_t which_dir)
{
  auto dst_idx_pair = planning_utils::findFirstIdxOutOfRange(planning_utils::extractPoses(lane),
                                                             pose.position, stop_dist, idx, which_dir);
  if(!dst_idx_pair.first)
  {
    ROS_ERROR("findFirstIdxOutOfRange: cannot find index");
    return std::make_tuple(false, -1, geometry_msgs::Pose());
  }

  // calculate actual stop position
  auto dst_idx = dst_idx_pair.second;
  ROS_DEBUG("idx: %d", idx);

  geometry_msgs::Pose res;
  if(dst_idx_pair.second == 0)
  {
    res = lane.points.front().pose;
  }
  else if(dst_idx_pair.second == (int32_t)(lane.points.size() - 1))
  {
    res = lane.points.back().pose;
  }
  else
  {
    geometry_msgs::Pose line_s;
    geometry_msgs::Pose line_e;

    if(which_dir == 1)
    {
      line_s = lane.points.at(dst_idx).pose;
      line_e = lane.points.at(dst_idx + 1).pose;
    }
    else
    {
      line_s = lane.points.at(dst_idx - 1).pose;
      line_e = lane.points.at(dst_idx).pose;
    }

    auto actual_pair =  planning_utils::findIntersectionWithLineCircle(line_s.position, line_e.position,
                                                                       pose.position, stop_dist, which_dir);
    if(!actual_pair.first)
    {
      ROS_ERROR("findIntersectionWithLineCircle: cannot find actual pos");
      return std::make_tuple(false, -1, geometry_msgs::Pose());
    }

    res.position = actual_pair.second;
    if(planning_utils::isDirectionForward(line_s, line_e))
    {
      res.orientation = planning_utils::getQuaternionFromYaw(
          atan2((line_e.position.y - line_s.position.y), (line_e.position.x - line_s.position.x))
      );
    }
    else
    {
      res.orientation = planning_utils::getQuaternionFromYaw(
          atan2((line_s.position.y - line_e.position.y), (line_s.position.x - line_e.position.x))
      );
    }
  }

  ROS_DEBUG("actual: (%lf, %lf)", res.position.x, res.position.y);

  return std::make_tuple(true, dst_idx, res);
}


}  // namespace planning_utils

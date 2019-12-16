/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

#include <velocity_planner/velocity_planner_utils.hpp>
#include <velocity_planner/interpolate.h>

namespace vpu
{
double square(const double &a)
{
  return a * a;
}
double calcSquaredDist2d(const geometry_msgs::Point &a, const geometry_msgs::Point &b)
{
  return square(a.x - b.x) + square(a.y - b.y);
}
double calcSquaredDist2d(const geometry_msgs::Pose &a, const geometry_msgs::Pose &b)
{
  return square(a.position.x - b.position.x) + square(a.position.y - b.position.y);
}
double calcSquaredDist2d(const geometry_msgs::PoseStamped &a, const geometry_msgs::PoseStamped &b)
{
  return square(a.pose.position.x - b.pose.position.x) + square(a.pose.position.y - b.pose.position.y);
}
double calcSquaredDist2d(const autoware_planning_msgs::TrajectoryPoint &a, const autoware_planning_msgs::TrajectoryPoint &b)
{
  return square(a.pose.position.x - b.pose.position.x) + square(a.pose.position.y - b.pose.position.y);
}


int calcClosestWaypoint(const autoware_planning_msgs::Trajectory &traj, const geometry_msgs::Point &point)
{
  double dist_squared_min = std::numeric_limits<double>::max();
  int idx_min = -1;

  for (int i = 0; i < (int)traj.points.size(); ++i)
  {
    const double dx = traj.points.at(i).pose.position.x - point.x;
    const double dy = traj.points.at(i).pose.position.y - point.y;
    const double dist_squared = dx * dx + dy * dy;
    if (dist_squared < dist_squared_min)
    {
      dist_squared_min = dist_squared;
      idx_min = i;
    }
  }
  return idx_min;
}

int calcClosestWaypoint(const autoware_planning_msgs::Trajectory &trajectory, const geometry_msgs::Pose &pose)
{
  return calcClosestWaypoint(trajectory, pose.position);
}

bool extractPathAroundIndex(const autoware_planning_msgs::Trajectory &trajectory, const int index, const double &ahead_length,
                            const double &behind_length, autoware_planning_msgs::Trajectory &extracted_base_trajectory)
{
  if (trajectory.points.size() == 0 || (int)trajectory.points.size() - 1 < index || index < 0)
  {
    return false;
  }

  double dist_sum_tmp = 0.0;

  // calc ahead distance
  int ahead_index = trajectory.points.size() - 1;
  for (int i = index; i < (int)trajectory.points.size() - 1; ++i)
  {
    dist_sum_tmp += std::sqrt(calcSquaredDist2d(trajectory.points.at(i), trajectory.points.at(i + 1)));
    if (dist_sum_tmp > ahead_length)
    {

      ahead_index = i + 1;
      break;
    }
  }


  // calc behind distance
  dist_sum_tmp = 0.0;
  int behind_index = 0;
  for (int i = index; i > 0; --i)
  {
    dist_sum_tmp += std::sqrt(calcSquaredDist2d(trajectory.points.at(i), trajectory.points[i - 1]));
    if (dist_sum_tmp > behind_length)
    {
      behind_index = i - 1;
      break;
    }
  }


  // extruct trajectory
  extracted_base_trajectory.points.clear();
  for (int i = behind_index; i < ahead_index + 1; ++i)
  {
    extracted_base_trajectory.points.push_back(trajectory.points.at(i));
  }
  extracted_base_trajectory.header = trajectory.header;

  return true;
}

bool linearInterpPath(const autoware_planning_msgs::Trajectory &base_trajectory, const int resample_num, autoware_planning_msgs::Trajectory &resampled_trajectory)
{
  if (base_trajectory.points.size() == 0)
  {
    ROS_ERROR("[linearInterpPath] : base_trajectory.points.size() is zero. return false.");
    return false;
  }
  resampled_trajectory.points.clear();
  const double ds = 1.0 / (double)resample_num;
  autoware_planning_msgs::TrajectoryPoint tp;

  for (int i = 0; i < (int)base_trajectory.points.size() - 1; ++i)
  {
    double s = 0.0;
    for (int j = 0; j < resample_num; ++j)
    {
      tp = base_trajectory.points.at(i); // copy properties

      const geometry_msgs::Pose tp0_pose = base_trajectory.points.at(i).pose;
      const geometry_msgs::Pose tp1_pose = base_trajectory.points.at(i + 1).pose;
      const geometry_msgs::Twist tp0_twist = base_trajectory.points.at(i).twist;
      const geometry_msgs::Twist tp1_twist = base_trajectory.points.at(i + 1).twist;
      const geometry_msgs::Accel tp0_accel = base_trajectory.points.at(i).accel;
      const geometry_msgs::Accel tp1_accel = base_trajectory.points.at(i + 1).accel;
      tp.pose.position.x = (1.0 - s) * tp0_pose.position.x + s * tp1_pose.position.x;
      tp.pose.position.y = (1.0 - s) * tp0_pose.position.y + s * tp1_pose.position.y;
      tp.pose.position.z = (1.0 - s) * tp0_pose.position.z + s * tp1_pose.position.z;
      tf2::Quaternion q0, q1;
      tf2::fromMsg(tp0_pose.orientation, q0);
      tf2::fromMsg(tp1_pose.orientation, q1);
      tp.pose.orientation = tf2::toMsg(q0.slerp(q1, s));
      // tp.twist.linear.x = (1.0 - s) * tp0_twist.linear.x + s * tp1_twist.linear.x;
      tp.twist.linear.x = tp0_twist.linear.x;
      tp.twist.linear.y = (1.0 - s) * tp0_twist.linear.y + s * tp1_twist.linear.y;
      tp.twist.linear.z = (1.0 - s) * tp0_twist.linear.z + s * tp1_twist.linear.z;
      tp.twist.angular.x = (1.0 - s) * tp0_twist.angular.x + s * tp1_twist.angular.x;
      tp.twist.angular.y = (1.0 - s) * tp0_twist.angular.y + s * tp1_twist.angular.y;
      tp.twist.angular.z = (1.0 - s) * tp0_twist.angular.z + s * tp1_twist.angular.z;
      tp.accel.linear.x = (1.0 - s) * tp0_accel.linear.x + s * tp1_accel.linear.x;
      tp.accel.linear.y = (1.0 - s) * tp0_accel.linear.y + s * tp1_accel.linear.y;
      tp.accel.linear.z = (1.0 - s) * tp0_accel.linear.z + s * tp1_accel.linear.z;
      tp.accel.angular.x = (1.0 - s) * tp0_accel.angular.x + s * tp1_accel.angular.x;
      tp.accel.angular.y = (1.0 - s) * tp0_accel.angular.y + s * tp1_accel.angular.y;
      tp.accel.angular.z = (1.0 - s) * tp0_accel.angular.z + s * tp1_accel.angular.z;
      resampled_trajectory.points.push_back(tp);
      s += ds;
    }
  }

  resampled_trajectory.points.push_back(base_trajectory.points.back());

  return true;
}

double calcLengthOnWaypoints(const autoware_planning_msgs::Trajectory &path, const int idx1, const int idx2)
{
  if (idx1 == idx2)  // zero distance
    return 0.0;

  if (idx1 < 0 || idx2 < 0 || (int)path.points.size() - 1 < idx1 || (int)path.points.size() - 1 < idx2)
  {
    std::cerr << "vpu::calcLengthOnWaypoints(): invalid index" << std::endl;
    return 0.0;
  }

  const int idx_from = std::min(idx1, idx2);
  const int idx_to = std::max(idx1, idx2);
  double dist_sum = 0.0;
  for (int i = idx_from; i < idx_to; ++i)
  {
    dist_sum += std::sqrt(calcSquaredDist2d(path.points.at(i), path.points.at(i+1)));
  }
  return dist_sum;
}

bool scalingVelocitWithStopPoint(const autoware_planning_msgs::Trajectory &trajectory, const int &self_idx, const int &stop_idx, autoware_planning_msgs::Trajectory &trajectory_scaled)
{
  if (trajectory.points.size() == 0) {
    std::cerr << "[vpu::scalingVelocitWithyStopPoint] trajectory size is zero, return false" << std::endl;
    return false;
  }
  if (stop_idx <= self_idx )
  {
    /* over stop point, return all zero velocity */
    trajectory_scaled = trajectory;
    for(auto &tp : trajectory_scaled.points)
    {
      tp.twist.linear.x = 0.0;
    }
    return true;
  }

  std::vector<double> v_arr, s_arr;
  double arclength = 0.0;
  v_arr.push_back(trajectory.points.at(self_idx).twist.linear.x);
  s_arr.push_back(arclength);

  int idx_v0 = -1;
  for (unsigned int i = self_idx + 1; i < trajectory.points.size(); ++i)
  {
    const autoware_planning_msgs::TrajectoryPoint tp = trajectory.points.at(i);
    const autoware_planning_msgs::TrajectoryPoint tp_prev = trajectory.points.at(i - 1);
    const double v = tp.twist.linear.x;
    v_arr.push_back(v);
    arclength += calcSquaredDist2d(tp.pose, tp_prev.pose);
    s_arr.push_back(arclength);
    if (std::fabs(v) < 0.01 && idx_v0 == -1) // zero velocity index
    {
      idx_v0 = (int)i;
    }
  }
  if (idx_v0 == -1)
  {
    idx_v0 = (int)trajectory.points.size() - 1;
  }
  

  const double distance_to_stop = calcLengthOnWaypoints(trajectory, self_idx, stop_idx);
  const double distance_to_v0 = calcLengthOnWaypoints(trajectory, self_idx, idx_v0);
  const double ratio = distance_to_v0 / std::max(1.0E-5, distance_to_stop);
  // printf("to_stop = %f, to_v0 = %f, ratio = %f\n", distance_to_stop, distance_to_v0, ratio);

  std::vector<double> s_arr_scaled;
  for (unsigned int i = 0; i < s_arr.size(); ++i)
  {
    if (s_arr.at(i) * ratio < distance_to_v0)
    {
      s_arr_scaled.push_back(s_arr.at(i) * ratio);  // scaled arc-length
    }
    else
    {
      break;
    }
  }


  std::vector<double> v_arr_scaled;
  if (s_arr_scaled.size() != 0)
  {
    if (!LinearInterpolate::interpolate(s_arr, v_arr, s_arr_scaled, v_arr_scaled))
      return false;
  }


  trajectory_scaled = trajectory;
  for (unsigned int i = 0; i < v_arr_scaled.size(); ++i)
  {
    trajectory_scaled.points.at(self_idx + i).twist.linear.x = v_arr_scaled.at(i);
  }
  
  for (int i = self_idx + (int)v_arr_scaled.size(); i < (int)trajectory.points.size(); ++i) 
  {
    trajectory_scaled.points.at(i).twist.linear.x = 0.0;
  }

  // for (int i = 0; i < trajectory.points.size(); ++i)
  // {
  //   printf("i = %d, ", i);

  //   printf("base_vel = %f", trajectory.points.at(i).twist.linear.x);

  //   if (self_idx <= i && i < s_arr.size() + self_idx)
  //     printf("s_arr = %f, ", s_arr.at(i - self_idx));
  //   else
  //     printf("s_arr = ----, ");

  //   if (self_idx <= i && i < v_arr.size() + self_idx)
  //     printf("v_arr = %f, ", v_arr.at(i - self_idx));
  //   else
  //     printf("v_arr = ----, ");

  //   if (self_idx <= i && i < s_arr_scaled.size() + self_idx)
  //     printf("s_arr_scaled = %f, ", s_arr_scaled.at(i - self_idx));
  //   else
  //     printf("s_arr_scaled = ----, ");

  //   if (self_idx <= i && i < v_arr_scaled.size() + self_idx)
  //     printf("v_arr_scaled = %f, ", v_arr_scaled.at(i - self_idx));
  //   else
  //     printf("v_arr_scaled = ----, ");

  //   printf("result_vel = %f\n", trajectory_scaled.points.at(i).twist.linear.x);
  // }

  return true;

}

void calcWaypointsArclength(const autoware_planning_msgs::Trajectory &path, std::vector<double> &arclength)
{
  double dist = 0.0;
  arclength.clear();
  arclength.push_back(dist);
  for (unsigned int i = 1; i < path.points.size(); ++i)
  {
    const autoware_planning_msgs::TrajectoryPoint tp = path.points.at(i);
    const autoware_planning_msgs::TrajectoryPoint tp_prev = path.points.at(i - 1);
    dist += std::sqrt(calcSquaredDist2d(tp.pose, tp_prev.pose));
    arclength.push_back(dist);
  }
}

void zeroVelocity(autoware_planning_msgs::Trajectory &trajectory)
{
  for (auto &tp : trajectory.points)
  {
    tp.twist.linear.x = 0.0;
  }
  return;
}

bool calcStopDistWithConstantJerk(const double &v0, const double &a0, const double &s_lim, const double &v_end,
                                    double &t1, double &t2, double &stop_dist)
{
  double s = std::fabs(s_lim);
  const double t2_squared = (v0 - v_end) / s + 0.5 * a0 * a0 / s / s;
  if (t2_squared < 0.0)
  {
    ROS_WARN("[calcStopDistWithConstantJerk] t2_squared = %f < 0, something wronggg!, v0 = %f, a0 = %f, s_lim = %f, "
              "v_end = %f",
              t2_squared, v0, a0, s_lim, v_end);
    return false;
  }
  t2 = std::sqrt(t2_squared);
  t1 = t2 + a0 / s;

  if (t1 < 0.0 || t2 < 0.0)
  {
    s = -s;
    t2 = std::sqrt(t2_squared);
    t1 = t2 + a0 / s;
  }

  const double x1 = -s * t1 * t1 * t1 / 6 + a0 * t1 * t1 / 2 + v0 * t1;
  const double v1 = -s * t1 * t1 / 2 + a0 * t1 + v0;
  const double a1 = -s * t1 + a0;
  if (x1 < 0.0 || v1 < 0.0)
  {
    // ROS_WARN("[calcStopDistWithConstantJerk] x1 = %f, v1 = %f, maybe initial acceleration or velocity is too small : "
    //           "v0 = %f, a0 = %f, s_lim = %f, t1 = %f, t2 = %f",
    //           x1, v1, v0, a0, s_lim, t1, t2);
    return false;
  }
  const double x2 = s * t2 * t2 * t2 / 6.0 + a1 * t2 * t2 / 2.0 + v1 * t2 + x1;
  stop_dist = x2;
  return true;
}

bool calcStopVelocityWithConstantJerk(const double &v0, const double &a0, const double &planning_jerk, const double &t1,
                                      const double &t2, const int &start_idx, autoware_planning_msgs::Trajectory &trajectory, 
                                      std::vector<double> &a_arr_out, std::vector<double> &s_arr_out)
{
  auto calc_xv = [](const double &v0, const double &a0, const double &s_lim, const double &t, const double &t1,
                    const double &t2, double &x, double &v, double &a, double &jerk) {
    double s;
    if (std::fabs(t2 - t1) < 1.0E-9 /* a0 == 0 */)
    {
      s = v0 > 0 ? std::fabs(s_lim) : -std::fabs(s_lim);
    }
    else
    {
      s =  -a0 / (t2 - t1);
    }

    if (0 <= t && t <= t1)
    {
      x = -s * t * t * t / 6 + a0 * t * t / 2 + v0 * t;
      v = -s * t * t / 2 + a0 * t + v0;
      a = -s * t + a0;
      jerk = -s;
    }
    else if (t1 < t && t <= t1 + t2)
    {
      const double x1 = -s * t1 * t1 * t1 / 6 + a0 * t1 * t1 / 2 + v0 * t1;
      const double v1 = -s * t1 * t1 / 2 + a0 * t1 + v0;
      const double a1 = -s * t1 + a0;
      const double t_tmp = t - t1;
      x = s * t_tmp * t_tmp * t_tmp / 6 + a1 * t_tmp * t_tmp / 2 + v1 * t_tmp + x1;
      v = s * t_tmp * t_tmp / 2 + a1 * t_tmp + v1;
      a = s * t_tmp + a1;
      jerk = s;
    }
    // printf("x = %f, v = %f, a = %f, jerk = %f, t = %f, t1 = %f, t2 = %f, a0 = %f, v0 = %f\n", x, v, a, jerk, t, t1, t2, a0, v0);
  };
  const double t_total = t1 + t2;
  std::vector<double> t_arr, x_arr, v_arr, a_arr, s_arr;
  const double dt = 0.1;
  double x(0.0), v(0.0), a(0.0), s(0.0);
  for (double t = 0.0; t < t_total; t += dt)
  {
    t_arr.push_back(t);
    calc_xv(v0, a0, std::fabs(planning_jerk), t, t1, t2, x, v, a, s);
    if (x_arr.size() > 0)
    {
      if (x < x_arr.back())
      {
        ROS_ERROR("[calcStopVelocityWithConstantJerk] : x isn't increase, something wrong!");
      }
    }
    if (x < -0.0001 || v < -0.0001)
    {
      ROS_ERROR("x = %f, v = %f, wierd conditigon. break", x, v);
      break;
    }
    else
    {
      x_arr.push_back(std::max(x, 0.0));
      v_arr.push_back(std::max(v, 0.0));
      a_arr.push_back(a);
      s_arr.push_back(s);
    }
  }
  t_arr.push_back(t_total);
  calc_xv(v0, a0, std::fabs(planning_jerk), t_total, t1, t2, x, v, a, s);
  if (x < -0.0001 || v < -0.0001)
  {
    ROS_ERROR("end: x = %f, v = %f, wierd conditigon. break", x, v);
  }
  else
  {
    if (x_arr.size() > 0)
    {
      if (x > x_arr.back())
      {
        x_arr.push_back(std::max(x, 0.0));
        v_arr.push_back(std::max(v, 0.0));
        a_arr.push_back(a);
        s_arr.push_back(s);
      }
    }
    else
    {
      x_arr.push_back(std::max(x, 0.0));
      v_arr.push_back(std::max(v, 0.0));
      a_arr.push_back(a);
      s_arr.push_back(s);
    }
  }

  if (x_arr.size() == 0 || v_arr.size() == 0 || a_arr.size() == 0 || s_arr.size() == 0)
  {
    ROS_ERROR("[calcStopVelocityWithConstantJerk] x_arr.size() is zero. something wrong.");
    return false;
  }

  /* 距離ベクトルの計算 */
  double dist = 0.0;
  std::vector<double> arclength;
  arclength.push_back(dist);
  for (unsigned int i = start_idx + 1; i < trajectory.points.size(); ++i)
  {
    const autoware_planning_msgs::TrajectoryPoint tp = trajectory.points.at(i);
    const autoware_planning_msgs::TrajectoryPoint tp_prev = trajectory.points.at(i - 1);
    dist += std::sqrt(calcSquaredDist2d(tp.pose, tp_prev.pose));
    if (dist > x_arr.back())
      break;
    arclength.push_back(dist);
  }

  /* x(t_i), v(t_i)のベクトルが求まったので、x-v の関係から、trajectoryのxにおけるvの値を線形補間で求める */
  std::vector<double> vel_at_x;
  if (!LinearInterpolate::interpolate(x_arr, v_arr, arclength, vel_at_x) || 
      !LinearInterpolate::interpolate(x_arr, a_arr, arclength, a_arr_out) || 
      !LinearInterpolate::interpolate(x_arr, s_arr, arclength, s_arr_out))
  {
    ROS_ERROR("[calcStopVelocityWithConstantJerk] interpolation error");
    ROS_ERROR("*** v0 = %f, a0 = %f, s = %f, t1 = %f, t2 = %f", v0, a0, s, t1, t2);
    for (int i = 0; i < (int)x_arr.size(); ++i) {
      ROS_ERROR("i = %d, in_x.at(i) = %f, in_v.at(i) = %f",i, x_arr.at(i), v_arr.at(i));
    }
    for (int i = 0; i < (int)arclength.size(); ++i) {
      ROS_ERROR("i = %d, ref_x.at(i) = %f, out_v.at(i) = %f",i, arclength.at(i), vel_at_x.at(i));
    }
    return false;
  }

  for (int i = 0; i < (int)vel_at_x.size(); ++i)
  {
    trajectory.points.at(start_idx + i).twist.linear.x = vel_at_x.at(i);
  }
  for (int i = start_idx + (int)vel_at_x.size(); i < (int)trajectory.points.size(); ++i)
  {
    // trajectory.points.at(i).twist.linear.x = vel_at_x.back();
    trajectory.points.at(i).twist.linear.x = v_arr.back();
  }

  return true;
}

void mininumVelocityFilter(const double &min_vel, autoware_planning_msgs::Trajectory &trajectory)
{
  for (auto &tp : trajectory.points)
  {
    if (tp.twist.linear.x < min_vel)
      tp.twist.linear.x = min_vel;
  }
}

void maximumVelocityFilter(const double &max_vel, autoware_planning_msgs::Trajectory &trajectory)
{
  const double abs_max_vel = std::fabs(max_vel);
  for (auto &tp : trajectory.points)
  {
    if (tp.twist.linear.x > abs_max_vel)
      tp.twist.linear.x = abs_max_vel;
    else if (tp.twist.linear.x < -abs_max_vel)
      tp.twist.linear.x = -abs_max_vel;
  }
}
void multiplyConstantToTrajectoryVelocity(const double &scalar, autoware_planning_msgs::Trajectory &trajectory)
{
  for (auto &tp : trajectory.points)
  {
    tp.twist.linear.x *= scalar;
  }
}

void insertZeroVelocityAfterIdx(const int &stop_idx, autoware_planning_msgs::Trajectory &trajectory)
{
  if(stop_idx < 0)
    return;

  for (int i = stop_idx; i < (int)trajectory.points.size(); ++i)
  {
    trajectory.points.at(i).twist.linear.x = 0.0;
  }
}

double getVx(const autoware_planning_msgs::Trajectory &trajectory, const int &i)
{
  return trajectory.points.at(i).twist.linear.x;
}

double getForwardAcc(const autoware_planning_msgs::Trajectory &trajectory, const int &i)
{
  const double v_curr_ref = trajectory.points.at(i).twist.linear.x;
  const double dist_next = std::sqrt(vpu::calcSquaredDist2d(trajectory.points.at(i + 1), trajectory.points.at(i)));
  const double dt_next = std::max(0.0001, dist_next) / std::max(v_curr_ref, 0.2 /* to avoid zero devide */);
  const double v_next_ref = trajectory.points.at(i + 1).twist.linear.x;
  return (v_next_ref - v_curr_ref) / dt_next;
}

double getDurationToNextIdx(const autoware_planning_msgs::Trajectory &trajectory, const double &v, const int &i)
{
    const double dist_prev = std::sqrt(vpu::calcSquaredDist2d(trajectory.points.at(i + 1), trajectory.points.at(i)));
    return  std::max(0.0001, dist_prev) / std::max(v, 0.2 /* to avoid zero devide */);
}

bool searchZeroVelocityIdx(const autoware_planning_msgs::Trajectory &trajectory, int &idx)
{
  for (unsigned int i = 0; i < trajectory.points.size(); ++i)
  {
    if (std::fabs(vpu::getVx(trajectory, i)) < 1.0E-3)
    {
      idx = i;
      return true;
    }
  }
  return false;
}

bool calcWaypointsCurvature(const autoware_planning_msgs::Trajectory &trajectory, const unsigned int &idx_dist, std::vector<double> &k_arr)
{
  k_arr.clear();
  if (trajectory.points.size() < 2 * idx_dist) 
  {
    ROS_ERROR("[calcWaypointsCurvature] cannot calc curvature idx_dist = %df, trajectory.size() = %lu", idx_dist, trajectory.points.size());
    for (unsigned int i = 0; i < trajectory.points.size(); ++i)
    {
      k_arr.push_back(0.0);
    }
    return false;
  }
  

  /* calculate curvature by circle fitting from three points */
  geometry_msgs::Point p1, p2, p3;
  for (unsigned int i = idx_dist; i < trajectory.points.size() - idx_dist; ++i)
  {
    p1.x = trajectory.points.at(i - idx_dist).pose.position.x;
    p2.x = trajectory.points.at(i).pose.position.x;
    p3.x = trajectory.points.at(i + idx_dist).pose.position.x;
    p1.y = trajectory.points.at(i - idx_dist).pose.position.y;
    p2.y = trajectory.points.at(i).pose.position.y;
    p3.y = trajectory.points.at(i + idx_dist).pose.position.y;
    double den = std::max(calcSquaredDist2d(p1, p2) * calcSquaredDist2d(p2, p3) * calcSquaredDist2d(p3, p1), 0.0001);
    double curvature = 2.0 * ((p2.x - p1.x) * (p3.y - p1.y) - (p2.y - p1.y) * (p3.x - p1.x)) / den;
    k_arr.push_back(curvature);
  }

  // for debug
  if (k_arr.size() == 0)
  {
    ROS_ERROR("[calcWaypointsCurvature] k_arr.size() = 0, somthing wrong. pls check.");
    return false;
  }

  /* first and last curvature is copied from next value */
  for (unsigned int i = 0; i < idx_dist; ++i)
  {
    k_arr.insert(k_arr.begin(), k_arr.front());
    k_arr.push_back(k_arr.back());
  }
  return true;
}

bool backwardAccelerationFilterForStopPoint(const double &accel, autoware_planning_msgs::Trajectory &trajectory)
{
  int zero_index;
  bool exist_stop_point = vpu::searchZeroVelocityIdx(trajectory, zero_index);
  ROS_WARN("[STOP ACC FILTER] exist_stop_point = %d ", exist_stop_point);
  if (!exist_stop_point)
    return true;

  for (int i = zero_index - 1; i >= 0; --i)
  {
    const double dist = std::sqrt(calcSquaredDist2d(trajectory.points.at(i), trajectory.points.at(i + 1)));
    const double v0 = trajectory.points.at(i + 1).twist.linear.x;
    const double v1 = std::sqrt(v0 * v0 + 2.0 * std::fabs(accel) * dist);
    printf("i = %d, dist = %3.3f, v0 = %3.3f, v1 = %3.3f\n", i, dist, v0, v1);
    if (trajectory.points.at(i).twist.linear.x > v1)
    {
      trajectory.points.at(i).twist.linear.x = v1;
      ROS_WARN("[STOP ACC FILTER] v(%d): %3.3f -> %3.3f ", i, trajectory.points.at(i).twist.linear.x, v1);
    }
    else
    {
      break;
    }
  }
  return true;
}

}  // namespace vpu





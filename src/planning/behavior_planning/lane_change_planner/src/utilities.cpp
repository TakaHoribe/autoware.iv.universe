/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
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

#include <lane_change_planner/utilities.h>

namespace lane_change_planner
{
namespace util
{
using autoware_planning_msgs::PathWithLaneId;
using autoware_perception_msgs::PredictedPath;

double l2Norm(const geometry_msgs::Vector3 vector)
{
  return std::sqrt(std::pow(vector.x, 2) + std::pow(vector.y, 2) + std::pow(vector.z, 2));
}

Eigen::Vector3d convertToEigenPt(const geometry_msgs::Point geom_pt)
{
  return Eigen::Vector3d(geom_pt.x, geom_pt.y, geom_pt.z);
}

bool convertToFrenetCoordinate3d(const std::vector<geometry_msgs::Point>& linestring,
                                 const geometry_msgs::Point search_point_geom, FrenetCoordinate3d* frenet_coordinate)
{
  if (linestring.empty())
  {
    return false;
  }

  auto prev_geom_pt = linestring.front();
  const auto search_pt = convertToEigenPt(search_point_geom);
  bool found = false;
  double min_distance = std::numeric_limits<double>::max();
  double accumulated_length = 0;

  for (const auto& geom_pt : linestring)
  {
    const auto start_pt = convertToEigenPt(prev_geom_pt);
    const auto end_pt = convertToEigenPt(geom_pt);

    const auto line_segment = end_pt - start_pt;
    const double line_segment_length = line_segment.norm();
    const auto direction = line_segment / line_segment_length;
    const auto start2search_pt = (search_pt - start_pt);

    double tmp_length = direction.dot(start2search_pt);
    if (tmp_length >= 0 && tmp_length <= line_segment_length)
    {
      double tmp_distance = direction.cross(start2search_pt).norm();
      if (tmp_distance < min_distance)
      {
        found = true;
        min_distance = tmp_distance;
        frenet_coordinate->distance = tmp_distance;
        frenet_coordinate->length = accumulated_length + tmp_length;
      }
    }
    accumulated_length += line_segment_length;
    prev_geom_pt = geom_pt;
  }
  return found;
}

std::vector<geometry_msgs::Point> convertToGeometryPointArray(const PathWithLaneId& path)
{
  std::vector<geometry_msgs::Point> converted_path;
  converted_path.reserve(path.points.size());
  for (const auto& point_with_id : path.points)
  {
    converted_path.push_back(point_with_id.point.pose.position);
  }
  return converted_path;
}

PredictedPath convertToPredictedPath(const PathWithLaneId& path, const geometry_msgs::Twist& vehicle_twist,
                                     const geometry_msgs::Pose& vehicle_pose)
{
  PredictedPath predicted_path;
  predicted_path.path.reserve(path.points.size());
  if (path.points.empty())
  {
    return predicted_path;
  }

  const auto& geometry_points = convertToGeometryPointArray(path);
  FrenetCoordinate3d vehicle_pose_frenet;
  convertToFrenetCoordinate3d(geometry_points, vehicle_pose.position, &vehicle_pose_frenet);

  ros::Time start_time = ros::Time::now();
  double vehicle_speed = std::abs(vehicle_twist.linear.x);
  double accumulated_distance = 0;

  auto prev_pt = path.points.front();
  for (size_t i = 0; i < path.points.size(); i++)
  {
    auto pt = path.points.at(i);
    FrenetCoordinate3d pt_frenet;
    if (!convertToFrenetCoordinate3d(geometry_points, pt.point.pose.position, &pt_frenet))
    {
      continue;
    }
    double frenet_distance = pt_frenet.length - vehicle_pose_frenet.length;
    double travel_time = frenet_distance / vehicle_speed;
    auto time_stamp = start_time + ros::Duration(travel_time);

    geometry_msgs::PoseWithCovarianceStamped predicted_pose;
    predicted_pose.header.stamp = time_stamp;
    predicted_pose.pose.pose.position = pt.point.pose.position;
    predicted_pose.pose.pose.orientation = pt.point.pose.orientation;
    predicted_path.path.push_back(predicted_pose);
    prev_pt = pt;
  }
  return predicted_path;
}

geometry_msgs::Pose lerpByPose(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2, const double t)
{
  tf2::Transform tf_transform1, tf_transform2;
  tf2::fromMsg(p1, tf_transform1);
  tf2::fromMsg(p2, tf_transform2);
  const auto& tf_point = tf2::lerp(tf_transform1.getOrigin(), tf_transform2.getOrigin(), t);
  const auto& tf_quaternion = tf2::slerp(tf_transform1.getRotation(), tf_transform2.getRotation(), t);

  geometry_msgs::Pose pose;
  pose.position = tf2::toMsg(tf_point, pose.position);
  pose.orientation = tf2::toMsg(tf_quaternion);
  return pose;
}

bool lerpByTimeStamp(const PredictedPath& path, const ros::Time& t, geometry_msgs::Pose* lerped_pt)
{
  if (lerped_pt == nullptr)
  {
    ROS_WARN_STREAM(__func__ << " failed due to nullptr pt");
  }
  if (path.path.empty())
  {
    ROS_WARN_STREAM("Empty path. Failed to interplate path by time!");
    return false;
  }
  if (t < path.path.front().header.stamp || t > path.path.back().header.stamp)
  {
    ROS_WARN_STREAM("failed to interpolate path by time!"
                    << std::endl
                    << "path start time: " << path.path.front().header.stamp << std::endl
                    << "path end time  : " << path.path.back().header.stamp << std::endl
                    << "query time     : " << t);
    return false;
  }

  for (size_t i = 1; i < path.path.size(); i++)
  {
    const auto& pt = path.path.at(i);
    const auto& prev_pt = path.path.at(i - 1);
    if (t < pt.header.stamp)
    {
      const auto duration = pt.header.stamp - prev_pt.header.stamp;
      const auto off_set = t - prev_pt.header.stamp;
      const auto ratio = off_set.toSec() / duration.toSec();
      *lerped_pt = lerpByPose(prev_pt.pose.pose, pt.pose.pose, ratio);
      return true;
    }
  }

  ROS_ERROR_STREAM("Something failed in function: " << __func__);
  return false;
}

double getDistance3d(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
  return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2) + std::pow(p1.z - p2.z, 2));
}

double getDistanceBetweenPredictedPaths(const PredictedPath& path1, const PredictedPath& path2, const double resolution,
                                        const double duration)
{
  ros::Duration t_delta(resolution);
  ros::Duration prediction_duration(duration);
  double min_distance = std::numeric_limits<double>::max();
  ros::Time start_time = ros::Time::now();
  ros::Time end_time = ros::Time::now() + prediction_duration;

  for (auto t = start_time; t < end_time; t += t_delta)
  {
    geometry_msgs::Pose p1, p2;
    if (!lerpByTimeStamp(path1, t, &p1))
    {
      continue;
    }
    if (!lerpByTimeStamp(path2, t, &p2))
    {
      continue;
    }
    double distance = getDistance3d(p1.position, p2.position);
    if (distance < min_distance)
    {
      min_distance = distance;
    }
  }
  return min_distance;
}

std::vector<size_t> filterObjectsByLanelets(const autoware_perception_msgs::DynamicObjectArray& objects,
                                            const lanelet::ConstLanelets& target_lanelets)
{
  std::vector<size_t> indices;
  for (size_t i = 0; i < objects.objects.size(); i++)
  {
    const auto& obj_position = objects.objects.at(i).state.pose_covariance.pose.position;
    lanelet::BasicPoint2d obj_position2d(obj_position.x, obj_position.y);
    for (const auto& llt : target_lanelets)
    {
      double distance = boost::geometry::distance(llt.polygon2d().basicPolygon(), obj_position2d);
      if (distance < std::numeric_limits<double>::epsilon())
      {
        indices.push_back(i);
      }
    }
  }
  return indices;
}

PathWithLaneId removeOverlappingPoints(const PathWithLaneId& input_path)
{
  PathWithLaneId filtered_path;
  for (const auto& pt : input_path.points)
  {
    if (filtered_path.points.empty())
    {
      filtered_path.points.push_back(pt);
      continue;
    }
    if (getDistance3d(filtered_path.points.back().point.pose.position, pt.point.pose.position) <
        std::numeric_limits<double>::epsilon())
    {
      filtered_path.points.back().lane_ids.push_back(pt.lane_ids.front());
    }
    else
    {
      filtered_path.points.push_back(pt);
    }
  }
  return filtered_path;
}

template <typename T>
bool exists(std::vector<T> vec, T element)
{
  return std::find(vec.begin(), vec.end(), element) != vec.end();
}

bool setGoal(const double search_radius_range, const double search_rad_range, const PathWithLaneId& input,
             const geometry_msgs::Pose& goal, const int64_t goal_lane_id, PathWithLaneId* output_ptr)
{
  try
  {
    size_t min_dist_index;
    double min_dist = std::numeric_limits<double>::max();
    double goal_z;
    {
      bool found = false;
      for (size_t i = 0; i < input.points.size(); ++i)
      {
        const double x = input.points.at(i).point.pose.position.x - goal.position.x;
        const double y = input.points.at(i).point.pose.position.y - goal.position.y;
        const double z = input.points.at(i).point.pose.position.z - goal.position.z;
        const double dist = sqrt(x * x + y * y);
        if (dist < search_radius_range && dist < min_dist && exists(input.points.at(i).lane_ids, goal_lane_id))
        {
          min_dist_index = i;
          min_dist = dist;
          found = true;
        }
      }
      if (!found)
      {
        return false;
      }
    }

    size_t min_dist_out_of_range_index;
    {
      for (size_t i = min_dist_index; 0 <= i; --i)
      {
        const double x = input.points.at(i).point.pose.position.x - goal.position.x;
        const double y = input.points.at(i).point.pose.position.y - goal.position.y;
        const double z = input.points.at(i).point.pose.position.z - goal.position.z;
        goal_z = input.points.at(i).point.pose.position.z;
        const double dist = sqrt(x * x + y * y);
        min_dist_out_of_range_index = i;
        if (search_radius_range < dist)
        {
          break;
        }
        if (i == 0)
        {
          break;
        }
      }
    }
    autoware_planning_msgs::PathPointWithLaneId refined_goal;
    refined_goal.point.pose = goal;
    refined_goal.point.pose.position.z = goal_z;
    refined_goal.point.twist.linear.x = 0.0;

    autoware_planning_msgs::PathPointWithLaneId pre_refined_goal;
    double roll, pitch, yaw;
    pre_refined_goal.point.pose = goal;
    tf2::Quaternion tf2_quaternion(goal.orientation.x, goal.orientation.y, goal.orientation.z, goal.orientation.w);
    tf2::Matrix3x3 tf2_matrix(tf2_quaternion);
    tf2_matrix.getRPY(roll, pitch, yaw);
    pre_refined_goal.point.pose.position.x -= std::cos(yaw);
    pre_refined_goal.point.pose.position.y -= std::sin(yaw);
    pre_refined_goal.point.pose.position.z = goal_z;
    pre_refined_goal.point.twist.linear.x = input.points.at(min_dist_out_of_range_index).point.twist.linear.x;

    for (size_t i = 0; i <= min_dist_out_of_range_index; ++i)
    {
      output_ptr->points.push_back(input.points.at(i));
    }
    output_ptr->points.push_back(pre_refined_goal);
    output_ptr->points.push_back(refined_goal);
    return true;
  }
  catch (std::out_of_range& ex)
  {
    std::cout << ex.what() << std::endl;
    return false;
  }
}

PathWithLaneId refinePath(const double search_radius_range, const double search_rad_range, const PathWithLaneId& input,
                          const geometry_msgs::Pose& goal, const int64_t goal_lane_id)
{
  PathWithLaneId filtered_path, path_with_goal;
  filtered_path = removeOverlappingPoints(input);
  if (setGoal(search_radius_range, search_rad_range, filtered_path, goal, goal_lane_id, &path_with_goal))
  {
    return path_with_goal;
  }
  else
  {
    return filtered_path;
  }
}

}  // namespace util
}  // namespace lane_change_planner

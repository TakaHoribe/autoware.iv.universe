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

#include "pure_pursuit/pure_pursuit_viz.h"

// display the next waypoint by markers.
std::unique_ptr<visualization_msgs::Marker> displayNextWaypoint(const geometry_msgs::Point& position) {
  std::unique_ptr<visualization_msgs::Marker> marker(new visualization_msgs::Marker);
  marker->header.frame_id = MAP_FRAME;
  marker->header.stamp = ros::Time();
  marker->ns = "next_waypoint_marker";
  marker->id = 0;
  marker->type = visualization_msgs::Marker::SPHERE;
  marker->action = visualization_msgs::Marker::ADD;
  marker->pose.position = position;
  double size = 0.3;
  marker->scale.x = size;
  marker->scale.y = size;
  marker->scale.z = size;
  marker->color.a = 1.0;
  marker->color.r = 0.0;
  marker->color.g = 0.0;
  marker->color.b = 1.0;
  marker->frame_locked = true;
  return marker;
}

// display the next target by markers.
std::unique_ptr<visualization_msgs::Marker> displayNextTarget(const geometry_msgs::Point& target) {
  std::unique_ptr<visualization_msgs::Marker> marker(new visualization_msgs::Marker);
  marker->header.frame_id = MAP_FRAME;
  marker->header.stamp = ros::Time();
  marker->ns = "next_target_marker";
  marker->id = 0;
  marker->type = visualization_msgs::Marker::SPHERE;
  marker->action = visualization_msgs::Marker::ADD;
  marker->pose.position = target;
  std_msgs::ColorRGBA green;
  green.a = 1.0;
  green.b = 0.0;
  green.r = 0.0;
  green.g = 1.0;
  marker->color = green;
  double size = 0.3;
  marker->scale.x = size;
  marker->scale.y = size;
  marker->scale.z = size;
  marker->frame_locked = true;
  return marker;
}

// generate the locus of pure pursuit
std::vector<geometry_msgs::Point> generateTrajectoryCircle(const geometry_msgs::Point& target,
                                                           const geometry_msgs::Pose& curr_pose) {
  std::vector<geometry_msgs::Point> traj_circle_array;
  double radius = planning_utils::calcRadius(target, curr_pose);
  const double theta_base = M_PI / 2.0;
  const double theta_range_min = M_PI * 4 / 10;
  const double theta_range_max = M_PI * 6 / 10;
  const double coeff = 0.005;

  int32_t i = 0;
  while (1) {
    double theta = theta_base - i * coeff;
    if (theta < theta_range_min) break;

    // set a relative point of circumference
    geometry_msgs::Point p;
    p.x = -radius * cos(theta);
    p.y = -radius * sin(theta) + radius;
    p.z = target.z;

    // transform to vehicle plane
    geometry_msgs::Point transformed_p = planning_utils::transformToAbsoluteCoordinate2D(p, curr_pose);

    traj_circle_array.push_back(transformed_p);
    i++;
  }

  // reverse vector
  std::reverse(traj_circle_array.begin(), traj_circle_array.end());

  i = 1;
  while (1) {
    double theta = theta_base + i * coeff;
    if (theta > theta_range_max) break;

    // set a relative point of circumference
    geometry_msgs::Point p;
    p.x = -radius * cos(theta);
    p.y = -radius * sin(theta) + radius;
    p.z = target.z;

    // transform to vehicle plane
    geometry_msgs::Point transformed_p = planning_utils::transformToAbsoluteCoordinate2D(p, curr_pose);

    traj_circle_array.push_back(transformed_p);
    i++;
  }

  return traj_circle_array;
}
// display the locus of pure pursuit by markers.
std::unique_ptr<visualization_msgs::Marker> displayTrajectoryCircle(
    const std::vector<geometry_msgs::Point>& traj_circle_array) {
  std::unique_ptr<visualization_msgs::Marker> traj_circle(new visualization_msgs::Marker);
  traj_circle->header.frame_id = MAP_FRAME;
  traj_circle->header.stamp = ros::Time();
  traj_circle->ns = "trajectory_circle_marker";
  traj_circle->id = 0;
  traj_circle->type = visualization_msgs::Marker::LINE_STRIP;
  traj_circle->action = visualization_msgs::Marker::ADD;

  std_msgs::ColorRGBA white;
  white.a = 1.0;
  white.b = 1.0;
  white.r = 1.0;
  white.g = 1.0;

  for (auto el : traj_circle_array) {
    traj_circle->points.push_back(el);
    traj_circle->colors.push_back(white);
  }

  traj_circle->scale.x = 0.05;
  traj_circle->frame_locked = true;
  return traj_circle;
}

// display the search radius by markers.
std::unique_ptr<visualization_msgs::Marker> displaySearchRadius(const geometry_msgs::Point& current_pose,
                                                                const double search_radius) {
  std::unique_ptr<visualization_msgs::Marker> marker(new visualization_msgs::Marker);
  marker->header.frame_id = MAP_FRAME;
  marker->header.stamp = ros::Time();
  marker->ns = "search_radius_marker";
  marker->id = 0;
  marker->type = visualization_msgs::Marker::SPHERE;
  marker->action = visualization_msgs::Marker::ADD;
  marker->pose.position = current_pose;
  marker->scale.x = search_radius * 2;
  marker->scale.y = search_radius * 2;
  marker->scale.z = 1.0;
  marker->color.a = 0.1;
  marker->color.r = 1.0;
  marker->color.g = 0.0;
  marker->color.b = 0.0;
  marker->frame_locked = true;
  return marker;
}

visualization_msgs::MarkerArray displayControlTrajectory(const autoware_planning_msgs::Trajectory& trajectory) {
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker;
  marker.header.frame_id = MAP_FRAME;
  marker.header.stamp = ros::Time();
  marker.ns = "control_trajectory";
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.color.r = 1.0;
  marker.color.g = 1.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.005;
  marker.scale.z = 0.005;
  marker.frame_locked = true;

  for (size_t i = 0; i < trajectory.points.size(); i++) {
    marker.id = i;
    marker.pose = trajectory.points.at(i).pose;
    ma.markers.push_back(marker);
  }

  marker.ns = "control_trajectory_num";
  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.z = 0.05;
  for (size_t i = 0; i < trajectory.points.size(); i++) {
    marker.id = i;
    marker.pose = trajectory.points.at(i).pose;
    marker.text = std::to_string(i);
    ma.markers.push_back(marker);
  }

  return ma;
}

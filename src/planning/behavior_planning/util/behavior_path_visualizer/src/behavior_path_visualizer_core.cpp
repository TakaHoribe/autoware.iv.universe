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

#include <behavior_path_visualizer/behavior_path_visualizer.h>

BehaviorPathVisualizer::BehaviorPathVisualizer() : pnh_("~")
{
  path_subscriber_ = pnh_.subscribe("input/path_with_lane_id", 1, &BehaviorPathVisualizer::pathCallback, this);
  marker_publisher_ = pnh_.advertise<visualization_msgs::MarkerArray>("output/path_with_lane_id_marker", 1);
}

void BehaviorPathVisualizer::pathCallback(const autoware_planning_msgs::PathWithLaneId& path) const
{
  if (marker_publisher_.getNumSubscribers() < 1)
  {
    return;
  }

  marker_publisher_.publish(convertPathToMarker(path));
}

visualization_msgs::MarkerArray BehaviorPathVisualizer::convertPathToMarker(const autoware_planning_msgs::PathWithLaneId& path) const
{
  visualization_msgs::MarkerArray output_msg;
  {
    visualization_msgs::Marker marker;
    marker.header = path.header;
    marker.ns = "points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.scale.x = marker.scale.y = 0.1;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    for (size_t i = 0; i < path.points.size(); ++i)
    {
      marker.points.push_back(path.points.at(i).point.pose.position);
    }
    output_msg.markers.push_back(marker);
  }

  {
    visualization_msgs::Marker marker;
    marker.header = path.header;
    marker.ns = "line";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.05;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = ros::Duration(0.5);
    marker.color.a = 1.0;  // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    for (size_t i = 0; i < path.points.size(); ++i)
    {
      marker.points.push_back(path.points.at(i).point.pose.position);
    }
    output_msg.markers.push_back(marker);
  }

  {
    for (size_t i = 0; i < path.points.size(); ++i)
    {
      visualization_msgs::Marker marker;
      marker.header = path.header;
      marker.ns = "lane_id";
      marker.id = i;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.scale.z = 0.25;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose = path.points.at(i).point.pose;
      std::string text;
      if (path.points.at(i).lane_ids.empty())
        continue;
      for (size_t j = 0; j < path.points.at(i).lane_ids.size(); ++j)
      {
        text = text + std::string(std::to_string(path.points.at(i).lane_ids.at(j))) + std::string(",");
      }
      text.pop_back();
      marker.text = text;
      marker.lifetime = ros::Duration(0.5);
      marker.color.a = 1.0;  // Don't forget to set the alpha!
      marker.color.r = 1.0;
      marker.color.g = 1.0;
      marker.color.b = 1.0;
      output_msg.markers.push_back(marker);
    }
  }
  return output_msg;
}

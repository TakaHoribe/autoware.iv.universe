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

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <string>

namespace behavior_planning {
class Route2PathConverterNode
{
public:
  Route2PathConverterNode();

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Timer timer_;
  ros::Subscriber route_sub_;
  ros::Subscriber perception_pub_;
  ros::Subscriber pointcloud_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher path_pub_;
  

  std::shared_ptr<autoware_planning_msgs::Route> route_ptr_;
  std::shared_ptr<autoware_perception_msgs::DynamicObjectArray> perception_ptr_;
  std::shared_ptr<sensor_msgs::PointCloud2> pointcloud_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;

  void timerCallback(const ros::TimerEvent &e);
  void routeCallback(const autoware_planning_msgs::Route &input_route_msg);
  void perceptionCallback(const autoware_perception_msgs::DynamicObjectArray &input_perception_msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2 &input_pointcloud_msg);
  void mapCallback(const autoware_lanelet2_msgs::MapBin& input_map_msg);
  bool callback(const autoware_planning_msgs::Route &input_route_msg,
                const autoware_perception_msgs::DynamicObjectArray &input_perception_msg,
                const sensor_msgs::PointCloud2 &input_pointcloud_msg,
                autoware_planning_msgs::Path &output_path_msg);
};
}

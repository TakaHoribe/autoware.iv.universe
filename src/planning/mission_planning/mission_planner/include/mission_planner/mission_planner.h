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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <string>

class MissionPlanner
{
public:
  MissionPlanner();

private:
  bool is_graph_ready_;

  geometry_msgs::PoseStamped goal_pose_;
  geometry_msgs::PoseStamped start_pose_;

  std::string base_link_frame_;
  std::string map_frame_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher marker_publisher_;
  ros::Publisher route_publisher_;
  ros::Subscriber goal_subscriber_;
  ros::Subscriber map_subscriber_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  bool getEgoVehiclePose(geometry_msgs::PoseStamped* ego_vehicle_pose);
  void goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg);
  void mapCallback(const autoware_lanelet2_msgs::MapBin& msg);

  bool transformPose(const geometry_msgs::PoseStamped& input_pose, geometry_msgs::PoseStamped* output_pose,
                     const std::string target_frame);

  bool isRoutingGraphReady();
  autoware_planning_msgs::Route planRoute();
  void visualizeRoute(const autoware_planning_msgs::Route& route);
};

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

#ifndef MISSION_PLANNER_LANELET2_IMPL_MISSION_PLANNER_LANELET2_H
#define MISSION_PLANNER_LANELET2_IMPL_MISSION_PLANNER_LANELET2_H

// ROS
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

// Autoware
#include <autoware_lanelet2_msgs/MapBin.h>
#include <mission_planner/mission_planner_base.h>
#include <mission_planner/lanelet2_impl/route_handler.h>

// lanelet
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

// others
#include <string>
#include <unordered_map>

using LaneletToManuever = std::unordered_map<lanelet::Id, unsigned char>;

namespace mission_planner
{
class MissionPlannerLanelet2 : public MissionPlanner
{
public:
  MissionPlannerLanelet2();

private:
  bool is_graph_ready_;

  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;

  ros::Subscriber map_subscriber_;

  void mapCallback(const autoware_lanelet2_msgs::MapBin& msg);

  // virtual functions
  bool isRoutingGraphReady();
  autoware_planning_msgs::Route planRoute();
  void visualizeRoute(const autoware_planning_msgs::Route& route);

  // routing
  lanelet::ConstLanelets getMainLanelets(lanelet::routing::Route& route, RouteHandler& lanelet_sequence_finder);
  autoware_planning_msgs::Route createRouteMessage(lanelet::ConstLanelets main_path,
                                                   const RouteHandler& route_handler,
                                                   const LaneletToManuever& lane2maneuver);
  LaneletToManuever setManeuversToLanes(const lanelet::routing::Route& route, const lanelet::ConstLanelets main_lanes);
};
}  // namespace mission_planner

#endif  // MISSION_PLANNER_LANELET2_IMPL_MISSION_PLANNER_LANELET2_H

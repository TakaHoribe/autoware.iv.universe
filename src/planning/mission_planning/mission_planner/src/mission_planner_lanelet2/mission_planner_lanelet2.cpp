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

#include <mission_planner/lanelet2_impl/utility_functions.h>
#include <mission_planner/lanelet2_impl/mission_planner_lanelet2.h>
#include <mission_planner/lanelet2_impl/route_handler.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <unordered_set>

namespace mission_planner
{
MissionPlannerLanelet2::MissionPlannerLanelet2() : is_graph_ready_(false)
{
  map_subscriber_ = nh_.subscribe("lanelet_map_bin", 10, &MissionPlannerLanelet2::mapCallback, this);
}

void MissionPlannerLanelet2::mapCallback(const autoware_lanelet2_msgs::MapBin& msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  is_graph_ready_ = true;
}

bool MissionPlannerLanelet2::isRoutingGraphReady()
{
  return (is_graph_ready_);
}

void MissionPlannerLanelet2::visualizeRoute(const autoware_planning_msgs::Route& route)
{
  lanelet::ConstLanelets route_lanelets;
  lanelet::ConstLanelets end_lanelets;
  lanelet::ConstLanelets normal_lanelets;
  lanelet::ConstLanelets goal_lanelets;

  for (const auto& route_section : route.route_sections)
  {
    for (const auto& lane_id : route_section.lane_ids)
    {
      auto lanelet = lanelet_map_ptr_->laneletLayer.get(lane_id);
      route_lanelets.push_back(lanelet);
      if (route_section.preferred_lane_id == lane_id)
      {
        goal_lanelets.push_back(lanelet);
      }
      else if (exists(route_section.continued_lane_ids, lane_id))
      {
        normal_lanelets.push_back(lanelet);
      }
      else
      {
        end_lanelets.push_back(lanelet);
      }
    }
  }

  std_msgs::ColorRGBA cl_route, cl_ll_borders, cl_end, cl_normal, cl_goal;
  setColor(&cl_route, 0.0, 0.7, 0.2, 0.7);
  setColor(&cl_goal, 0.0, 0.7, 0.7, 0.7);
  setColor(&cl_end, 0.0, 0.2, 0.7, 0.7);
  setColor(&cl_normal, 0.0, 0.7, 0.2, 0.7);
  setColor(&cl_ll_borders, 1.0, 1.0, 1.0, 0.999);

  visualization_msgs::MarkerArray route_marker_array;
  insertMarkerArray(&route_marker_array,
                    lanelet::visualization::laneletsBoundaryAsMarkerArray(route_lanelets, cl_ll_borders, false));
  insertMarkerArray(&route_marker_array,
                    lanelet::visualization::laneletsAsTriangleMarkerArray("route_lanelets", route_lanelets, cl_route));
  insertMarkerArray(&route_marker_array,
                    lanelet::visualization::laneletsAsTriangleMarkerArray("end_lanelets", end_lanelets, cl_end));
  insertMarkerArray(&route_marker_array, lanelet::visualization::laneletsAsTriangleMarkerArray(
                                             "normal_lanelets", normal_lanelets, cl_normal));
  insertMarkerArray(&route_marker_array,
                    lanelet::visualization::laneletsAsTriangleMarkerArray("goal_lanelets", goal_lanelets, cl_goal));
  marker_publisher_.publish(route_marker_array);
}

autoware_planning_msgs::Route MissionPlannerLanelet2::planRoute()
{
  autoware_planning_msgs::Route route_msg;

  lanelet::Lanelet start_lanelet;
  if (!getClosestLanelet(start_pose_.pose, lanelet_map_ptr_, &start_lanelet))
  {
    return route_msg;
  }

  lanelet::Lanelet goal_lanelet;
  if (!getClosestLanelet(goal_pose_.pose, lanelet_map_ptr_, &goal_lanelet))
  {
    return route_msg;
  }

  // get all possible lanes that can be used to reach goal (including all possible lane change)
  lanelet::Optional<lanelet::routing::Route> optional_route =
      routing_graph_ptr_->getRoute(start_lanelet, goal_lanelet, 0);
  if (!optional_route)
  {
    ROS_ERROR_STREAM("Failed to find a proper path!" << std::endl
                                                     << "start point: " << toString(start_pose_.pose) << std::endl
                                                     << "goal point: " << toString(goal_pose_.pose) << std::endl
                                                     << "start lane id: " << start_lanelet.id() << std::endl
                                                     << "goal lane id: " << goal_lanelet.id() << std::endl);
    return route_msg;
  }

  RouteHandler route_handler(lanelet_map_ptr_, routing_graph_ptr_, optional_route.get());
  auto main_lanelets = getMainLanelets(optional_route.get(), route_handler);

  //  create unordered map of
  //  first = lanelet id
  //  second = desired maneuver in given lanelet
  auto lane2maneuver = setManeuversToLanes(optional_route.get(), main_lanelets);

  //  create lane sequence
  route_msg = createRouteMessage(main_lanelets, route_handler, lane2maneuver);

  route_msg.header.stamp = ros::Time::now();
  route_msg.header.frame_id = map_frame_;
  route_msg.goal_pose = goal_pose_.pose;

  return route_msg;
}

lanelet::ConstLanelets MissionPlannerLanelet2::getMainLanelets(lanelet::routing::Route& route,
                                                               RouteHandler& route_handler)
{
  const auto shortest_path = route.shortestPath();
  auto lanelet_sequence = route_handler.getLaneletSequence(shortest_path.back());
  lanelet::ConstLanelets main_lanelets;
  while (!lanelet_sequence.empty())
  {
    main_lanelets.insert(main_lanelets.begin(), lanelet_sequence.begin(), lanelet_sequence.end());
    lanelet_sequence = route_handler.getPreviousLaneletSequence(lanelet_sequence);
  }
  return main_lanelets;
}

LaneletToManuever MissionPlannerLanelet2::setManeuversToLanes(const lanelet::routing::Route& route,
                                                              const lanelet::ConstLanelets main_lanes)
{
  LaneletToManuever maneuvers;
  for (const auto& main_lane : main_lanes)
  {
    maneuvers[main_lane.id()] = MANEUVER::KEEP;

    auto right_relations = lanelet::utils::query::getAllNeighborsRight(routing_graph_ptr_,
                                                                       main_lane);  // route_.rightRelations(main_lane);
    for (const auto& right : right_relations)
    {
      // vehicle must lane change left to go to main_lane
      maneuvers[right.id()] = MANEUVER::LEFT;
    }

    auto left_relations = lanelet::utils::query::getAllNeighborsLeft(routing_graph_ptr_,
                                                                     main_lane);  // route_.leftRelations(main_lane);
    for (const auto& left : left_relations)
    {
      // vehicle must lane change right to go to main_lane
      maneuvers[left.id()] = MANEUVER::RIGHT;
    }
  }

  return maneuvers;
}

autoware_planning_msgs::Route MissionPlannerLanelet2::createRouteMessage(const lanelet::ConstLanelets main_path,
                                                                         const RouteHandler& route_handler,
                                                                         const LaneletToManuever& lane2maneuver)
{
  autoware_planning_msgs::Route route_msg;

  if (main_path.empty())
    return route_msg;

  for (const auto& main_llt : main_path)
  {
    autoware_planning_msgs::RouteSection route_section_msg;
    lanelet::ConstLanelets route_section = route_handler.getNeighborsWithinRoute(main_llt);
    route_section_msg.preferred_lane_id = main_llt.id();
    for (const auto& section_llt : route_section)
    {
      route_section_msg.lane_ids.push_back(section_llt.id());
      lanelet::ConstLanelet next_lanelet;
      if (route_handler.getNextLaneletWithinRoute(section_llt, &next_lanelet))
      {
        route_section_msg.continued_lane_ids.push_back(section_llt.id());
      }
    }
    route_msg.route_sections.push_back(route_section_msg);
  }
  return route_msg;
}

}  // namespace mission_planner

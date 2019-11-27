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

#include <mission_planner/utility_functions.h>
#include <mission_planner/mission_planner.h>

#include <visualization_msgs/MarkerArray.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <lanelet2_routing/Route.h>

#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>

MissionPlanner::MissionPlanner() : is_graph_ready_(false), pnh_("~"), tf_listener_(tf_buffer_)
{
  pnh_.param<std::string>("map_frame", map_frame_, "map");
  pnh_.param<std::string>("base_link_frame", base_link_frame_, "base_link");

  goal_subscriber_ = nh_.subscribe("move_base_simple/goal", 10, &MissionPlanner::goalPoseCallback, this);
  map_subscriber_ = nh_.subscribe("lanelet_map_bin", 10, &MissionPlanner::mapCallback, this);

  route_publisher_ = nh_.advertise<autoware_planning_msgs::Route>("planning/mission_planning/route", 1, true);
  marker_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("planning/mission_planning/route_marker", 1, true);
}

void MissionPlanner::mapCallback(const autoware_lanelet2_msgs::MapBin& msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  is_graph_ready_ = true;
}

bool MissionPlanner::getEgoVehiclePose(geometry_msgs::PoseStamped* ego_vehicle_pose)
{
  geometry_msgs::PoseStamped base_link_origin;
  base_link_origin.header.frame_id = base_link_frame_;
  base_link_origin.pose.position.x = 0;
  base_link_origin.pose.position.y = 0;
  base_link_origin.pose.position.z = 0;
  base_link_origin.pose.orientation.x = 0;
  base_link_origin.pose.orientation.y = 0;
  base_link_origin.pose.orientation.z = 0;
  base_link_origin.pose.orientation.w = 0;

  //  transform base_link frame origin to map_frame to get vehicle positions
  return transformPose(base_link_origin, ego_vehicle_pose, map_frame_);
}

bool MissionPlanner::transformPose(const geometry_msgs::PoseStamped& input_pose,
                                   geometry_msgs::PoseStamped* output_pose, const std::string target_frame)
{
  geometry_msgs::TransformStamped transform;
  try
  {
    transform = tf_buffer_.lookupTransform(target_frame, input_pose.header.frame_id, ros::Time(0));
    tf2::doTransform(input_pose, *output_pose, transform);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_WARN("%s", ex.what());
    return false;
  }
}

bool MissionPlanner::isRoutingGraphReady()
{
  return (is_graph_ready_);
}

void MissionPlanner::goalPoseCallback(const geometry_msgs::PoseStampedConstPtr& goal_msg_ptr)
{
  if (!getEgoVehiclePose(&start_pose_))
  {
    ROS_ERROR("Failed to get ego vehicle pose in map frame. Aborting mission planning");
    return;
  }
  if (!transformPose(*goal_msg_ptr, &goal_pose_, map_frame_))
  {
    ROS_ERROR("Failed to get goal pose in map frame. Aborting mission planning");
    return;
  }
  if (!isRoutingGraphReady())
  {
    ROS_ERROR("RoutingGraph is not ready. Aborting mission planning");
    return;
  }

  autoware_planning_msgs::Route route = planRoute();
  if (!route.route_sections.empty())
  {
    ROS_INFO("Route successfuly planned. Publishing...");
    route_publisher_.publish(route);
    visualizeRoute(route);
  }
  else
  {
    ROS_ERROR("Calculated route is empty!");
  }
}

void MissionPlanner::visualizeRoute(const autoware_planning_msgs::Route& route)
{
  lanelet::ConstLanelets route_lanelets;
  for (const auto& route_section : route.route_sections)
  {
    for (const auto& lane_id : route_section.lane_ids)
    {
      route_lanelets.push_back(lanelet_map_ptr_->laneletLayer.get(lane_id));
    }
  }

  std_msgs::ColorRGBA cl_route, cl_ll_borders;
  setColor(&cl_route, 0.7, 0.2, 0.2, 0.3);
  setColor(&cl_ll_borders, 1.0, 1.0, 1.0, 1.0);

  visualization_msgs::MarkerArray route_marker_array;
  insertMarkerArray(&route_marker_array,
                    lanelet::visualization::laneletsBoundaryAsMarkerArray(route_lanelets, cl_ll_borders, false));
  insertMarkerArray(&route_marker_array,
                    lanelet::visualization::laneletsAsTriangleMarkerArray("route_lanelets", route_lanelets, cl_route));
  marker_publisher_.publish(route_marker_array);
}

autoware_planning_msgs::Route MissionPlanner::planRoute()
{
  autoware_planning_msgs::Route route;

  lanelet::Lanelet start_lanelet;
  if (!getClosestLanelet(start_pose_.pose, lanelet_map_ptr_, &start_lanelet))
  {
    return route;
  }

  lanelet::Lanelet goal_lanelet;
  if (!getClosestLanelet(goal_pose_.pose, lanelet_map_ptr_, &goal_lanelet))
  {
    return route;
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
    return route;
  }
  auto route_map = optional_route->laneletMap();

  //  get one possible path to reach goal.
  auto shortest_path = optional_route->shortestPath();

  //  creating a route message
  //  1. for each lane along the shortest path, get adjacent lanes in the same direction. (regardless of lanechange
  //     availability)
  //  2. compare with lanes in route_map to remove lanes that are not relevant to the route.
  lanelet::ConstLanelet previous_llt;
  for (const auto& llt : shortest_path)
  {
    // skip if the last manuever was lane change because the two lanes are in the same route_section
    if (traffic_rules_ptr_->canChangeLane(previous_llt, llt))
    {
      previous_llt = llt;
      continue;
    }
    auto lane_changeable_lanes = lanelet::utils::query::getAllNeighbors(routing_graph_ptr_, llt);
    autoware_planning_msgs::RouteSection route_section;
    for (const auto lane : lane_changeable_lanes)
    {
      if (route_map->laneletLayer.exists(lane.id()))
      {
        route_section.lane_ids.push_back(lane.id());
      }
    }
    route.route_sections.push_back(route_section);
    previous_llt = llt;
  }

  return route;
}

#pragma once
#include <behavior_velocity_planner/data_manager.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>

namespace behavior_planning
{
bool getDynemicObjects(std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> &objects);
bool getNoGroundPointcloud(std::shared_ptr<sensor_msgs::PointCloud2 const> &pointcloud);
bool getCurrentSelfVelocity(std::shared_ptr<geometry_msgs::TwistStamped const> &twist);
bool getBaselink2FrontLength(double &length);
bool isVehicleStopping();
bool getCurrentSelfPose(geometry_msgs::PoseStamped &pose);
bool getLaneletMap(lanelet::LaneletMapConstPtr &lanelet_map_pt,
                   lanelet::routing::RoutingGraphConstPtr &routing_graph_ptr);
}
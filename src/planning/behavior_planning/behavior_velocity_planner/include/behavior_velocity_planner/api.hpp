#pragma once
#include <behavior_velocity_planner/data_manager.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

namespace behavior_planning
{
bool getDynemicObjects(std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> &objects);
bool getNoGroundPointcloud(std::shared_ptr<sensor_msgs::PointCloud2 const> &pointcloud);
bool getNoGroundPointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pointcloud);
bool getCurrentSelfVelocity(std::shared_ptr<geometry_msgs::TwistStamped const> &twist);
bool getBaselink2FrontLength(double &length);
bool getVehicleWidth(double &width);
bool isVehicleStopping();
bool getCurrentSelfPose(geometry_msgs::PoseStamped &pose);
bool getLaneletMap(lanelet::LaneletMapConstPtr &lanelet_map_pt,
                   lanelet::routing::RoutingGraphConstPtr &routing_graph_ptr);
}
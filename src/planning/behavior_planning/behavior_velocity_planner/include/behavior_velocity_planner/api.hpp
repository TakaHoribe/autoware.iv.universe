#pragma once
#include <behavior_velocity_planner/data_manager.hpp>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>

namespace behavior_planning
{
bool getDynemicObjects(std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects)
{
    return SingletonDataManager::getInstance().getDynemicObjects(objects);
}

bool getNoGroundPointcloud(std::shared_ptr<sensor_msgs::PointCloud2 const> pointcloud)
{
    return SingletonDataManager::getInstance().getNoGroundPointcloud(pointcloud);
}

bool getCurrentSelfVelocity(std::shared_ptr<geometry_msgs::TwistStamped const> &twist){
    return SingletonDataManager::getInstance().getCurrentSelfVelocity(twist);
}

bool isVehicleStopping(){
    std::shared_ptr<geometry_msgs::TwistStamped const> twist_ptr;
    getCurrentSelfVelocity(twist_ptr);
    if (twist_ptr == nullptr)
    return false;
    return twist_ptr->twist.linear.x < 0.1 ? true : false;
}

bool getCurrentSelfPose(geometry_msgs::PoseStamped &pose)
{
    return SingletonDataManager::getInstance().getCurrentSelfPose(pose);
}

bool getLaneletMap(lanelet::LaneletMapConstPtr &lanelet_map_pt,
                   lanelet::routing::RoutingGraphConstPtr &routing_graph_ptr)
{
    return SingletonDataManager::getInstance().getLaneletMap(lanelet_map_pt, routing_graph_ptr);
}
}
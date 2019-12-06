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
#pragma once

#include <ros/ros.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2_ros/transform_listener.h>
#include <behavior_velocity_planner/node.hpp>
#include <memory>

namespace behavior_planning
{
class SelfPoseLinstener
{
public:
    SelfPoseLinstener();
    bool getSelfPose(geometry_msgs::Pose &self_pose, const std_msgs::Header &header);

private:
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

class SingletonDataManager
{
private:
    SingletonDataManager() = default;
    ~SingletonDataManager() = default;

public:
    SingletonDataManager(const SingletonDataManager &) = delete;
    SingletonDataManager &operator=(const SingletonDataManager &) = delete;
    SingletonDataManager(SingletonDataManager &&) = delete;
    SingletonDataManager &operator=(SingletonDataManager &&) = delete;
    static SingletonDataManager &getInstance()
    {
        static SingletonDataManager instance;
        return instance;
    }

    friend class BehaviorVelocityPlannerNode;

private:
    /* 
     * Cache
     */ 
    std::shared_ptr<autoware_perception_msgs::DynamicObjectArray> perception_ptr_;
    std::shared_ptr<sensor_msgs::PointCloud2> pointcloud_ptr_;
    lanelet::LaneletMapPtr lanelet_map_ptr_;
    lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
    lanelet::routing::RoutingGraphPtr routing_graph_ptr_;

    /*
     * SelfPoseLinstener
     */
    std::shared_ptr<SelfPoseLinstener> self_pose_listener_ptr_;
    void perceptionCallback(const autoware_perception_msgs::DynamicObjectArray &input_perception_msg);
    void pointcloudCallback(const sensor_msgs::PointCloud2 &input_pointcloud_msg);
    void mapCallback(const autoware_lanelet2_msgs::MapBin &input_map_msg);

public:
    bool getDynemicObjects(std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects);
    bool getNoGroundPointcloud(std::shared_ptr<sensor_msgs::PointCloud2 const> pointcloud);
    bool getCurrentSelfPose(geometry_msgs::PoseStamped &pose);
};

} // namespace behavior_planning
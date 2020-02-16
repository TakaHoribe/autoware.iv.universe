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

#include <memory>

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_traffic_light_msgs/TrafficLightStateArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

namespace behavior_planning {

class BehaviorVelocityPlannerNode;

struct PlannerData {
  autoware_perception_msgs::DynamicObjectArray::ConstPtr dynamic_objects;
  sensor_msgs::PointCloud2::ConstPtr no_ground_pointcloud_msg;
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr no_ground_pointcloud;
  autoware_traffic_light_msgs::TrafficLightState::ConstPtr traffic_light_state;

  geometry_msgs::PoseStamped::ConstPtr current_pose;
  geometry_msgs::TwistStamped::ConstPtr current_velocity;

  lanelet::LaneletMapPtr lanelet_map;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules;
  lanelet::routing::RoutingGraphPtr routing_graph;

  std::shared_ptr<double> wheel_base;
  std::shared_ptr<double> front_overhang;
  std::shared_ptr<double> vehicle_width;
  std::shared_ptr<double> base_link2front;

  bool is_vehicle_stopping;
};

class SingletonDataManager {
 private:
  SingletonDataManager() : tf_listener_(tf_buffer_) {}
  ~SingletonDataManager() = default;

 public:
  SingletonDataManager(const SingletonDataManager&) = delete;
  SingletonDataManager& operator=(const SingletonDataManager&) = delete;
  SingletonDataManager(SingletonDataManager&&) = delete;
  SingletonDataManager& operator=(SingletonDataManager&&) = delete;
  static SingletonDataManager& getInstance() {
    static SingletonDataManager instance;
    return instance;
  }

  friend class BehaviorVelocityPlannerNode;

 private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  PlannerData planner_data_;
  std::map<int, std::tuple<std_msgs::Header, autoware_traffic_light_msgs::TrafficLightState>> traffic_light_id_map_;

  void perceptionCallback(const autoware_perception_msgs::DynamicObjectArray::ConstPtr& msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);
  void mapCallback(const autoware_lanelet2_msgs::MapBin::ConstPtr& msg);
  void trafficLightStatesCallback(const autoware_traffic_light_msgs::TrafficLightStateArray::ConstPtr& msg);

 public:
  // setter
  void setWheelBase(const double& wheel_base);
  void setFrontOverhang(const double& front_overhang);
  void setVehicleWidth(const double& width);
  void setBaseLink2FrontLength();

  // getter
  PlannerData getPlannerData() {
    planner_data_.is_vehicle_stopping = isVehicleStopping();
    return planner_data_;
  }

  bool getSelfPose(geometry_msgs::Pose& self_pose, const std_msgs::Header& header);
  bool getDynemicObjects(autoware_perception_msgs::DynamicObjectArray::ConstPtr& objects);
  bool getNoGroundPointcloud(sensor_msgs::PointCloud2::ConstPtr& pointcloud);
  bool getTrafficLightState(const int id, std_msgs::Header& header,
                            autoware_traffic_light_msgs::TrafficLightState& traffic_light);
  bool getCurrentSelfPose(geometry_msgs::PoseStamped& pose);
  bool getCurrentSelfVelocity(geometry_msgs::TwistStamped::ConstPtr& twist);
  bool getLaneletMap(lanelet::LaneletMapConstPtr& lanelet_map_ptr,
                     lanelet::routing::RoutingGraphConstPtr& routing_graph_ptr);
  bool getWheelBase(double& wheel_base);
  bool getFrontOverhang(double& front_overhang);
  bool getVehicleWidth(double& width);
  bool getBaselink2FrontLength(double& length);

  bool isVehicleStopping();
};
}  // namespace behavior_planning

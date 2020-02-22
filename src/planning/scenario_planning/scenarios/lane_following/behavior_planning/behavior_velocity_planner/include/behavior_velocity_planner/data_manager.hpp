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

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_traffic_light_msgs/TrafficLightStateArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <tf2_ros/transform_listener.h>
#include <behavior_velocity_planner/node.hpp>
#include <memory>

namespace behavior_planning {
class SingletonDataManager {
 private:
  explicit SingletonDataManager();
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
  /*
   * Cache
   */
  std::shared_ptr<autoware_perception_msgs::DynamicObjectArray> perception_ptr_;
  std::shared_ptr<sensor_msgs::PointCloud2> pointcloud_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> vehicle_velocity_ptr_;
  lanelet::LaneletMapPtr lanelet_map_ptr_;
  lanelet::traffic_rules::TrafficRulesPtr traffic_rules_ptr_;
  lanelet::routing::RoutingGraphPtr routing_graph_ptr_;
  std::shared_ptr<double> wheel_base_ptr_;
  std::shared_ptr<double> front_overhang_ptr_;
  std::shared_ptr<double> vehicle_width_ptr_;
  std::map<int, std::tuple<std_msgs::Header, autoware_traffic_light_msgs::TrafficLightState>> traffic_light_id_map_;

  /*
   * Tf listener
   */
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  /*
   * Callback
   */
  void perceptionCallback(const autoware_perception_msgs::DynamicObjectArray& input_perception_msg);
  void pointcloudCallback(const sensor_msgs::PointCloud2& input_pointcloud_msg);
  void velocityCallback(const geometry_msgs::TwistStamped& input_twist_msg);
  void mapCallback(const autoware_lanelet2_msgs::MapBin& input_map_msg);
  void trafficLightStatesCallback(const autoware_traffic_light_msgs::TrafficLightStateArray& input_tl_states_msg);

 public:
  // setter
  void setWheelBase(const double& wheel_base);
  void setFrontOverhang(const double& front_overhang);
  void setVehicleWidth(const double& width);
  // getter
  bool getDynemicObjects(std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const>& objects);
  bool getNoGroundPointcloud(std::shared_ptr<sensor_msgs::PointCloud2 const>& pointcloud);
  bool getTrafficLightState(const int id, std_msgs::Header& header,
                            autoware_traffic_light_msgs::TrafficLightState& traffic_light);
  bool getCurrentSelfPose(geometry_msgs::PoseStamped& pose);
  bool getCurrentSelfVelocity(std::shared_ptr<geometry_msgs::TwistStamped const>& twist);
  bool getLaneletMap(lanelet::LaneletMapConstPtr& lanelet_map_ptr,
                     lanelet::routing::RoutingGraphConstPtr& routing_graph_ptr);
  bool getWheelBase(double& wheel_base);
  bool getFrontOverhang(double& front_overhang);
  bool getVehicleWidth(double& width);
  bool getTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                    const ros::Duration timeout, geometry_msgs::TransformStamped& transform_stamped);
};

}  // namespace behavior_planning
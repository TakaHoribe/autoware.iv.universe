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
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_listener.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

#include <behavior_velocity_planner/scene_module_interface.hpp>

#include <string>
#include <memory>

namespace behavior_planning {

class BehaviorVelocityPlannerManager
{
public:
  BehaviorVelocityPlannerManager();
  bool callback(const autoware_planning_msgs::PathWithLaneId &input_path_msg,
                autoware_planning_msgs::PathWithLaneId &output_path_msg);

private:
  std::vector<std::shared_ptr<SceneConditionInterface>> v_scene_condition_;
  std::vector<std::shared_ptr<SceneModuleInterface>> v_scene_module_;
};
}

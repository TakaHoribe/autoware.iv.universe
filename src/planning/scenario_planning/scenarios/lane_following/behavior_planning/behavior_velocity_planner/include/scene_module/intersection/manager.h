#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/intersection/scene.h>
#include <scene_module/scene_module_interface.h>

class IntersectionModuleManager : public SceneModuleManagerInterface {
 public:
  IntersectionModuleManager() : SceneModuleManagerInterface(getModuleName()) {}

  const char* getModuleName() override { return "intersection"; }

 private:
  void launchNewModules(const autoware_planning_msgs::PathWithLaneId& path) override;

  std::function<bool(const std::shared_ptr<SceneModuleInterface>&)> getModuleExpiredFunction(
      const autoware_planning_msgs::PathWithLaneId& path) override;
};
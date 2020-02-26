#pragma once

#include <string>
#include <unordered_map>

#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>
#include <scene_module/traffic_light/scene.h>

class TrafficLightModuleManager : public SceneModuleManagerInterface {
 public:
  TrafficLightModuleManager() : SceneModuleManagerInterface(getModuleName()) {}

  const char* getModuleName() override { return "traffic_light"; }
  void launchNewModules(const autoware_planning_msgs::PathWithLaneId& path) override;

 private:
  std::function<bool(const std::shared_ptr<SceneModuleInterface>&)> getModuleExpiredFunction(
      const autoware_planning_msgs::PathWithLaneId& path) override;
};

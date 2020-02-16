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
#include <scene_module/traffic_light/debug.h>
#include <scene_module/traffic_light/scene.h>

class TrafficLightModuleManager : public SceneModuleManagerInterface {
 public:
  const char* getModuleName() override { return "TrafficLight"; }
  void launchNewModules(const autoware_planning_msgs::PathWithLaneId& path) override;
  void deleteExpiredModules(const autoware_planning_msgs::PathWithLaneId& path) override;

  void debug() override { debugger_.publish(); }
  TrafficLightDebugMarkersManager debugger_;  // TODO: remove
};

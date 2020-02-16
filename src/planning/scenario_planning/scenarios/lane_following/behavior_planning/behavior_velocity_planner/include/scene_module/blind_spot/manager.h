#pragma once

#include <set>
#include <string>

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/blind_spot/debug.h>
#include <scene_module/blind_spot/scene.h>
#include <scene_module/scene_module_interface.h>

class BlindSpotModuleManager : public SceneModuleManagerInterface {
 public:
  const char* getModuleName() override { return "BlindSpot"; }
  void launchNewModules(const autoware_planning_msgs::PathWithLaneId& path) override;
  void deleteExpiredModules(const autoware_planning_msgs::PathWithLaneId& path) override;

  void debug() override {
    // directly publishing...
  }
  BlindSpotModuleDebugger debugger_;  // TODO: remove
};

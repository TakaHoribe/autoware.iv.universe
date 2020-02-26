#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/momentary_stop/scene.h>
#include <scene_module/scene_module_interface.h>

class MomentaryStopModuleManager : public SceneModuleManagerInterface {
 public:
  MomentaryStopModuleManager() : SceneModuleManagerInterface(getModuleName()) {}

  const char* getModuleName() override { return "momentary_stop"; }
  void launchNewModules(const autoware_planning_msgs::PathWithLaneId& path) override;

 private:
  std::function<bool(const std::shared_ptr<SceneModuleInterface>&)> getModuleExpiredFunction(
      const autoware_planning_msgs::PathWithLaneId& path) override;
};

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

namespace behavior_planning {

class TrafficLightModuleManager : public SceneModuleManagerInterface {
 public:
  bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                      std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) override;

  void debug() override { debugger_.publish(); }
  TrafficLightDebugMarkersManager debugger_;  // TODO: remove

  bool isRegistered(const lanelet::TrafficLight& traffic_light);
  void registerTask(const lanelet::TrafficLight& traffic_light);
  void unregisterTask(const lanelet::TrafficLight& traffic_light);

 private:
  std::set<int64_t> registered_task_id_set_;
};

}  // namespace behavior_planning

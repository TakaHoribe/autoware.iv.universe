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

namespace behavior_planning {

class BlindSpotModuleManager : public SceneModuleManagerInterface {
 public:
  bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                      std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) override;
  void debug() override {
    // directly publishing...
  }
  BlindSpotModuleDebugger debugger_;  // TODO: remove

  bool isRegistered(const int64_t lane_id);
  void registerTask(const int64_t lane_id);
  void unregisterTask(const int64_t lane_id);

 private:
  std::set<int64_t> registered_lane_id_set_;
};

}  // namespace behavior_planning

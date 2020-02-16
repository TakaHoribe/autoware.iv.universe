#pragma once

#include <memory>
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

#include <scene_module/intersection/debug.h>  // To be refactored
#include <scene_module/intersection/scene.h>
#include <scene_module/scene_module_interface.h>

namespace behavior_planning {

class IntersectionModuleManager : public SceneModuleManagerInterface {
 public:
  bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                      std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) override;
  void debug() override {
    // directly publishing...
  }
  IntersectionModuleDebugger debugger_;  // TODO: remove

  void unregisterTask(const int lane_id);

 private:
  std::vector<int> registered_lane_ids_;

  bool isRegistered(const int lane_id);
  void registerTask(const int lane_id);
};

}  // namespace behavior_planning

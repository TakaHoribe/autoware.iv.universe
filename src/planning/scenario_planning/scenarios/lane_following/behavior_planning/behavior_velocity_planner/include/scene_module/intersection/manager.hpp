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

#include <scene_module/intersection/debug.hpp>  // To be refactored
#include <scene_module/intersection/scene.hpp>
#include <scene_module/scene_module_interface.hpp>

namespace behavior_planning {

class IntersectionModuleManager : public SceneModuleManagerInterface {
 public:
  bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                      std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) override;
  IntersectionModuleDebugger debugger_;
  void unregisterTask(const int lane_id);

 private:
  std::vector<int> registered_lane_ids_;

  bool isRunning(const int lane_id);
  void registerTask(const int lane_id);
};

}  // namespace behavior_planning

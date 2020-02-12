#include <behavior_velocity_planner/planner_manager.hpp>

namespace behavior_planning {

void BehaviorVelocityPlannerManager::launchSceneModule(
    const std::shared_ptr<SceneModuleManagerInterface>& scene_module_manager_ptr) {
  scene_managers_ptr_.push_back(scene_module_manager_ptr);
}

bool BehaviorVelocityPlannerManager::callback(const autoware_planning_msgs::PathWithLaneId& input_path_msg,
                                              autoware_planning_msgs::PathWithLaneId& output_path_msg) {
  for (size_t i = 0; i < scene_managers_ptr_.size(); ++i) {
    if (!scene_managers_ptr_.at(i)->updateSceneModuleInstances(input_path_msg)) {
      return false;
    }
  }

  autoware_planning_msgs::PathWithLaneId input_path = input_path_msg;
  for (size_t i = 0; i < scene_managers_ptr_.size(); ++i) {
    autoware_planning_msgs::PathWithLaneId output_path;
    if (scene_managers_ptr_.at(i)->run(input_path, output_path)) input_path = output_path;
  }

  output_path_msg = input_path;

  return true;
}
}  // namespace behavior_planning

#include <behavior_velocity_planner/planner_manager.h>

void BehaviorVelocityPlannerManager::launchSceneModule(
    const std::shared_ptr<SceneModuleManagerInterface>& scene_module_manager_ptr) {
  scene_manager_ptrs_.push_back(scene_module_manager_ptr);
}

autoware_planning_msgs::PathWithLaneId BehaviorVelocityPlannerManager::planPathVelocity(
    const std::shared_ptr<const PlannerData>& planner_data,
    const autoware_planning_msgs::PathWithLaneId& input_path_msg) {
  autoware_planning_msgs::PathWithLaneId output_path_msg = input_path_msg;
  for (const auto& scene_manager_ptr : scene_manager_ptrs_) {
    scene_manager_ptr->updateSceneModuleInstances(planner_data, input_path_msg);
    scene_manager_ptr->modifyPathVelocity(&output_path_msg);
  }

  return output_path_msg;
}

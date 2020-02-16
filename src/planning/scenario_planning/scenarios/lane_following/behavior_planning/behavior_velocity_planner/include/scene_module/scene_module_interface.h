#pragma once

#include <memory>
#include <vector>

#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>

#include <behavior_velocity_planner/planner_data.h>

namespace behavior_planning {

class SceneModuleInterface {
 public:
  virtual bool endOfLife(
      const autoware_planning_msgs::PathWithLaneId& input) = 0;  // TODO: in the same class as startCondition

  virtual bool run(const autoware_planning_msgs::PathWithLaneId& input,
                   autoware_planning_msgs::PathWithLaneId& output) = 0;

  virtual void setPlannerData(const std::shared_ptr<const PlannerData>& planner_data) { planner_data_ = planner_data; }

 public:
  SceneModuleInterface() = default;
  virtual ~SceneModuleInterface() = default;

  std::shared_ptr<const PlannerData> planner_data_;
};

class SceneModuleManagerInterface {
 public:
  virtual bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                              std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) = 0;

  virtual void debug() {
    // do nothing by default
  }

 public:
  SceneModuleManagerInterface() = default;
  virtual ~SceneModuleManagerInterface() = default;

  virtual void updateSceneModuleInstances(const std::shared_ptr<const PlannerData>& planner_data,
                                          const autoware_planning_msgs::PathWithLaneId& input) {
    setPlannerData(planner_data);

    // create instance
    std::vector<std::shared_ptr<SceneModuleInterface>> scene_modules_ptr;
    startCondition(input, scene_modules_ptr);
    for (const auto& scene_module_ptr : scene_modules_ptr) {
      if (scene_module_ptr != nullptr) scene_module_ptrs_.push_back(scene_module_ptr);
    }

    // delete instance
    for (size_t i = 0; i < scene_module_ptrs_.size(); ++i) {
      if (scene_module_ptrs_.at(i)->endOfLife(input)) {
        scene_module_ptrs_.erase(scene_module_ptrs_.begin() + i);
        i += -1;
      }
    }
  }

  virtual bool run(const autoware_planning_msgs::PathWithLaneId& input,
                   autoware_planning_msgs::PathWithLaneId& output) {
    autoware_planning_msgs::PathWithLaneId input_path = input;

    for (const auto& scene_module_ptr : scene_module_ptrs_) {
      scene_module_ptr->setPlannerData(planner_data_);

      autoware_planning_msgs::PathWithLaneId output_path;
      if (scene_module_ptr->run(input_path, output_path)) input_path = output_path;
    }

    output = input_path;

    return true;
  }

  virtual void setPlannerData(const std::shared_ptr<const PlannerData>& planner_data) { planner_data_ = planner_data; }

 protected:
  std::vector<std::shared_ptr<SceneModuleInterface>> scene_module_ptrs_;
  std::shared_ptr<const PlannerData> planner_data_;
};

}  // namespace behavior_planning

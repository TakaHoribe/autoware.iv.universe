#pragma once

#include <memory>
#include <set>

#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>

#include <behavior_velocity_planner/planner_data.h>

// Debug
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

class SceneModuleInterface {
 public:
  explicit SceneModuleInterface(const int64_t module_id) : module_id_(module_id) {}
  virtual ~SceneModuleInterface() = default;

  virtual bool modifyPathVelocity(autoware_planning_msgs::PathWithLaneId* path) = 0;
  virtual visualization_msgs::MarkerArray createDebugMarkerArray() = 0;

  int64_t getModuleId() const { return module_id_; }
  void setPlannerData(const std::shared_ptr<const PlannerData>& planner_data) { planner_data_ = planner_data; }

 protected:
  const int64_t module_id_;
  std::shared_ptr<const PlannerData> planner_data_;
};

class SceneModuleManagerInterface {
 public:
  SceneModuleManagerInterface() = default;
  virtual ~SceneModuleManagerInterface() = default;

  virtual const char* getModuleName() = 0;
  virtual void launchNewModules(const autoware_planning_msgs::PathWithLaneId& path) = 0;
  virtual void deleteExpiredModules(const autoware_planning_msgs::PathWithLaneId& path) = 0;

  void updateSceneModuleInstances(const std::shared_ptr<const PlannerData>& planner_data,
                                  const autoware_planning_msgs::PathWithLaneId& path) {
    planner_data_ = planner_data;

    launchNewModules(path);
    deleteExpiredModules(path);
  }

  void modifyPathVelocity(autoware_planning_msgs::PathWithLaneId* path) {
    static const auto ns = std::string("output/debug/") + getModuleName();
    static ros::NodeHandle private_nh_("~");
    static ros::Publisher pub_debug_ = private_nh_.advertise<visualization_msgs::MarkerArray>(ns, 20);

    visualization_msgs::MarkerArray debug_marker_array;

    for (const auto& scene_module : scene_modules_) {
      scene_module->setPlannerData(planner_data_);
      scene_module->modifyPathVelocity(path);

      for (const auto& marker : scene_module->createDebugMarkerArray().markers) {
        debug_marker_array.markers.push_back(marker);
      }
    }

    pub_debug_.publish(debug_marker_array);
  }

 protected:
  bool isModuleRegistered(const int64_t module_id) { return registered_module_id_set_.count(module_id) != 0; }

  void registerModule(const std::shared_ptr<SceneModuleInterface>& scene_module) {
    ROS_INFO("register task: module = %s, id = %lu", getModuleName(), scene_module->getModuleId());
    scene_modules_.insert(scene_module);
    registered_module_id_set_.emplace(scene_module->getModuleId());
  }

  void unregisterModule(const std::shared_ptr<SceneModuleInterface>& scene_module) {
    ROS_INFO("unregister task: module = %s, id = %lu", getModuleName(), scene_module->getModuleId());
    scene_modules_.erase(scene_module);
    registered_module_id_set_.erase(scene_module->getModuleId());
  }

  std::set<std::shared_ptr<SceneModuleInterface>> scene_modules_;
  std::set<int64_t> registered_module_id_set_;

  std::shared_ptr<const PlannerData> planner_data_;
};

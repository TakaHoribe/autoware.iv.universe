#include <scene_module/crosswalk/manager.h>

#include <cmath>

namespace behavior_planning {

bool CrosswalkModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                            std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;
  const auto overall_graphs_ptr_ = planner_data_->overall_graphs;

  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      lanelet::ConstLanelet road_lanelet = lanelet_map_ptr->laneletLayer.get(input.points.at(i).lane_ids.at(j));
      std::vector<lanelet::ConstLanelet> crosswalks = overall_graphs_ptr_->conflictingInGraph(road_lanelet, 1);
      for (const auto& crosswalk : crosswalks) {
        if (!isRegistered(crosswalk))
          v_module_ptr.push_back(std::make_shared<CrosswalkModule>(this, crosswalk, input.points.at(i).lane_ids.at(j)));
        // -- debug code --
        std::vector<Eigen::Vector3d> points;
        for (const auto& lanelet_point : crosswalk.polygon3d()) {
          Eigen::Vector3d point;
          point << lanelet_point.x(), lanelet_point.y(), lanelet_point.z();
          points.push_back(point);
        }
        debugger_.pushCrosswalkPolygon(points);
        // ----------------
      }
    }
  }
  return true;
}

bool CrosswalkModuleManager::isRegistered(const lanelet::ConstLanelet& crosswalk) {
  return registered_task_id_set_.count(crosswalk.id()) != 0;
}

void CrosswalkModuleManager::registerTask(const lanelet::ConstLanelet& crosswalk) {
  ROS_INFO("Registered Crosswalk Task");
  registered_task_id_set_.emplace(crosswalk.id());
}

void CrosswalkModuleManager::unregisterTask(const lanelet::ConstLanelet& crosswalk) {
  ROS_INFO("Unregistered Crosswalk Task");
  registered_task_id_set_.erase(crosswalk.id());
}

}  // namespace behavior_planning

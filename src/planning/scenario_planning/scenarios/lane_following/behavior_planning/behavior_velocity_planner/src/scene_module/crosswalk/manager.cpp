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
        if (!isRunning(crosswalk))
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

bool CrosswalkModuleManager::isRunning(const lanelet::ConstLanelet& crosswalk) {
  if (task_id_direct_map_.count(crosswalk) == 0) return false;
  return true;
}

bool CrosswalkModuleManager::registerTask(const lanelet::ConstLanelet& crosswalk, const boost::uuids::uuid& uuid) {
  ROS_INFO("Registered Crosswalk Task");
  task_id_direct_map_.emplace(crosswalk, boost::lexical_cast<std::string>(uuid));
  task_id_reverse_map_.emplace(boost::lexical_cast<std::string>(uuid), crosswalk);
  return true;
}

bool CrosswalkModuleManager::unregisterTask(const boost::uuids::uuid& uuid) {
  ROS_INFO("Unregistered Crosswalk Task");
  task_id_direct_map_.erase(task_id_reverse_map_.at(boost::lexical_cast<std::string>(uuid)));
  task_id_reverse_map_.erase(boost::lexical_cast<std::string>(uuid));
  return true;
}

}  // namespace behavior_planning

#include <scene_module/crosswalk/manager.h>

namespace {

std::vector<lanelet::ConstLanelet> getCrosswalksOnPath(
    const autoware_planning_msgs::PathWithLaneId& path, const lanelet::LaneletMapPtr lanelet_map,
    const std::shared_ptr<const lanelet::routing::RoutingGraphContainer>& overall_graphs) {
  std::vector<lanelet::ConstLanelet> crosswalks;

  for (const auto& p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    const auto ll = lanelet_map->laneletLayer.get(lane_id);

    const auto conflicting_crosswalks = overall_graphs->conflictingInGraph(ll, 1);
    for (const auto& crosswalk : conflicting_crosswalks) {
      crosswalks.push_back(crosswalk);
    }
  }

  return crosswalks;
}

std::set<int64_t> getCrosswalkIdSetOnPath(
    const autoware_planning_msgs::PathWithLaneId& path, const lanelet::LaneletMapPtr lanelet_map,
    const std::shared_ptr<const lanelet::routing::RoutingGraphContainer>& overall_graphs) {
  std::set<int64_t> crosswalk_id_set;

  for (const auto& crosswalk : getCrosswalksOnPath(path, lanelet_map, overall_graphs)) {
    crosswalk_id_set.insert(crosswalk.id());
  }

  return crosswalk_id_set;
}

}  // namespace

void CrosswalkModuleManager::launchNewModules(const autoware_planning_msgs::PathWithLaneId& path) {
  for (const auto& crosswalk : getCrosswalksOnPath(path, planner_data_->lanelet_map, planner_data_->overall_graphs)) {
    const auto module_id = crosswalk.id();
    if (!isModuleRegistered(module_id)) {
      registerModule(std::make_shared<CrosswalkModule>(module_id, crosswalk));
    }
  }
}

void CrosswalkModuleManager::deleteExpiredModules(const autoware_planning_msgs::PathWithLaneId& path) {
  const auto crosswalk_id_set =
      getCrosswalkIdSetOnPath(path, planner_data_->lanelet_map, planner_data_->overall_graphs);

  for (const auto scene_module : scene_modules_) {
    if (crosswalk_id_set.count(scene_module->getModuleId()) == 0) {
      unregisterModule(scene_module);
    }
  }
}

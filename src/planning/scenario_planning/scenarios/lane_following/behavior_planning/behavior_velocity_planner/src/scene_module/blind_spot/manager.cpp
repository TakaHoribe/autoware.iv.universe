#include <scene_module/blind_spot/manager.h>

#include "utilization/util.h"

namespace {

std::vector<lanelet::ConstLanelet> getLaneletsOnPath(const autoware_planning_msgs::PathWithLaneId& path,
                                                     const lanelet::LaneletMapPtr lanelet_map) {
  std::vector<lanelet::ConstLanelet> lanelets;

  for (const auto& p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    lanelets.push_back(lanelet_map->laneletLayer.get(lane_id));
  }

  return lanelets;
}

std::set<int64_t> getLaneIdSetOnPath(const autoware_planning_msgs::PathWithLaneId& path) {
  std::set<int64_t> lane_id_set;

  for (const auto& p : path.points) {
    const auto lane_id = p.lane_ids.at(0);
    lane_id_set.insert(lane_id);
  }

  return lane_id_set;
}

}  // namespace

void BlindSpotModuleManager::launchNewModules(const autoware_planning_msgs::PathWithLaneId& path) {
  for (const auto& ll : getLaneletsOnPath(path, planner_data_->lanelet_map)) {
    const auto lane_id = ll.id();
    const auto module_id = lane_id;

    if (isModuleRegistered(module_id)) {
      continue;
    }

    // Is turning lane?
    const std::string turn_direction = ll.attributeOr("turn_direction", "else");
    if (turn_direction != "left" && turn_direction != "right") {
      continue;
    }

    registerModule(std::make_shared<BlindSpotModule>(module_id, lane_id, turn_direction));
  }
}

void BlindSpotModuleManager::deleteExpiredModules(const autoware_planning_msgs::PathWithLaneId& path) {
  const auto lane_id_set = getLaneIdSetOnPath(path);

  for (const auto scene_module : scene_modules_) {
    if (lane_id_set.count(scene_module->getModuleId()) == 0) {
      unregisterModule(scene_module);
    }
  }
}

#include <scene_module/blind_spot/manager.h>

#include "utilization/util.h"

namespace behavior_planning {

bool BlindSpotModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                            std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  /* get self pose */
  geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* search blind_spot tag */
  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      const int lane_id = input.points.at(i).lane_ids.at(j);
      lanelet::ConstLanelet lanelet_ij = lanelet_map_ptr->laneletLayer.get(lane_id);  // get lanelet layer
      std::string turn_direction = lanelet_ij.attributeOr("turn_direction", "else");  // get turn_direction

      if (!isRegistered(lane_id)) {
        // check blind_spot tag
        // no plan for straight
        if (turn_direction.compare("right") == 0 || turn_direction.compare("left") == 0) {
          // blind_spot tag is found. set module.
          v_module_ptr.push_back(std::make_shared<BlindSpotModule>(lane_id, turn_direction, this));
          registerTask(lane_id);
        }
      }
    }
  }

  return true;
}

bool BlindSpotModuleManager::isRegistered(const int64_t lane_id) { return registered_lane_id_set_.count(lane_id) != 0; }

void BlindSpotModuleManager::registerTask(const int64_t lane_id) { registered_lane_id_set_.emplace(lane_id); }

void BlindSpotModuleManager::unregisterTask(const int64_t lane_id) { registered_lane_id_set_.erase(lane_id); }

}  // namespace behavior_planning

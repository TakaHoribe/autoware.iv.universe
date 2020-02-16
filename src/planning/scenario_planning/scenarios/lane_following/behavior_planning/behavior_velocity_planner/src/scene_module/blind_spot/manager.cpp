#include <scene_module/blind_spot/manager.hpp>

#include "utilization/util.h"

namespace behavior_planning {

bool BlindSpotModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                            std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  /* get self pose */
  geometry_msgs::PoseStamped self_pose = *planner_data_->current_pose;

  /* get lanelet map */
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  /* search blind_spot tag */
  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      const int lane_id = input.points.at(i).lane_ids.at(j);
      lanelet::ConstLanelet lanelet_ij = lanelet_map_ptr->laneletLayer.get(lane_id);  // get lanelet layer
      std::string turn_direction = lanelet_ij.attributeOr("turn_direction", "else");  // get turn_direction

      if (!isRunning(lane_id)) {
        // check blind_spot tag
        if (turn_direction.compare("right") == 0 || turn_direction.compare("left") == 0)  // no plan for straight
        {
          // blind_spot tag is found. set module.
          v_module_ptr.push_back(std::make_shared<BlindSpotModule>(lane_id, turn_direction, this));
          registerTask(lane_id);
        }
      }
    }
  }

  return true;
}

bool BlindSpotModuleManager::isRunning(const int lane_id) {
  for (const auto& id : registered_lane_ids_) {
    if (id == lane_id) return true;
  }
  return false;
}

void BlindSpotModuleManager::registerTask(const int lane_id) { registered_lane_ids_.push_back(lane_id); }

void BlindSpotModuleManager::unregisterTask(const int lane_id) {
  const auto itr = std::find(registered_lane_ids_.begin(), registered_lane_ids_.end(), lane_id);
  if (itr == registered_lane_ids_.end())
    ROS_ERROR(
        "[BlindSpotModuleManager::unregisterTask()] : cannot remove task (lane_id = %d,"
        " registered_lane_ids_.size() = %lu)",
        lane_id, registered_lane_ids_.size());
  registered_lane_ids_.erase(itr);
}

}  // namespace behavior_planning

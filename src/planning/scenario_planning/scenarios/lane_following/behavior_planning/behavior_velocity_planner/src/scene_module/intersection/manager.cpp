#include <scene_module/intersection/manager.hpp>

#include <behavior_velocity_planner/api.hpp>  // To be refactored

#include "utilization/boost_geometry_helper.h"
#include "utilization/util.h"

namespace behavior_planning {

bool IntersectionModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                               std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  /* get self pose */
  geometry_msgs::PoseStamped self_pose;
  if (!getCurrentSelfPose(self_pose)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModuleManager::startCondition()] cannot get current self pose");
    return false;
  }

  /* get lanelet map */
  lanelet::LaneletMapConstPtr lanelet_map_ptr;               // objects info
  lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;  // route info
  if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr)) {
    ROS_WARN_DELAYED_THROTTLE(1.0, "[IntersectionModuleManager::startCondition()] cannot get lanelet map");
    return false;
  }

  /* search intersection tag */
  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      const int lane_id = input.points.at(i).lane_ids.at(j);
      lanelet::ConstLanelet lanelet_ij = lanelet_map_ptr->laneletLayer.get(lane_id);  // get lanelet layer
      std::string turn_direction = lanelet_ij.attributeOr("turn_direction", "else");  // get turn_direction

      if (!isRunning(lane_id)) {
        // check intersection tag
        if (turn_direction.compare("right") == 0 || turn_direction.compare("left") == 0 ||
            turn_direction.compare("straight") == 0) {
          // intersection tag is found. set module.
          v_module_ptr.push_back(std::make_shared<IntersectionModule>(lane_id, this));
          registerTask(lane_id);
        }
      }
    }
  }

  return true;
}

bool IntersectionModuleManager::isRunning(const int lane_id) {
  for (const auto& id : registered_lane_ids_) {
    if (id == lane_id) return true;
  }
  return false;
}

void IntersectionModuleManager::registerTask(const int lane_id) { registered_lane_ids_.push_back(lane_id); }

void IntersectionModuleManager::unregisterTask(const int lane_id) {
  const auto itr = std::find(registered_lane_ids_.begin(), registered_lane_ids_.end(), lane_id);
  if (itr == registered_lane_ids_.end())
    ROS_ERROR(
        "[IntersectionModuleManager::unregisterTask()] : cannot remove task (lane_id = %d,"
        " registered_lane_ids_.size() = %lu)",
        lane_id, registered_lane_ids_.size());
  registered_lane_ids_.erase(itr);
}

}  // namespace behavior_planning

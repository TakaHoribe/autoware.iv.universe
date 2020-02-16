#include <scene_module/traffic_light/manager.h>

#include <map>

#include <tf2/utils.h>

namespace behavior_planning {

bool TrafficLightModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                               std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  for (const auto& point : input.points) {
    for (const auto& lane_id : point.lane_ids) {
      std::vector<std::shared_ptr<const lanelet::TrafficLight>> tl_reg_elems =
          (lanelet_map_ptr->laneletLayer.get(lane_id)).regulatoryElementsAs<const lanelet::TrafficLight>();
      for (const auto& tl_reg_elem : tl_reg_elems) {
        if (!isRegistered(*tl_reg_elem)) {
          v_module_ptr.push_back(std::make_shared<TrafficLightModule>(this, tl_reg_elem, lane_id));
        }
      }
    }
  }

  return true;
}

bool TrafficLightModuleManager::isRegistered(const lanelet::TrafficLight& traffic_light) {
  const auto tl_stop_line_opt = traffic_light.stopLine();

  if (!tl_stop_line_opt) {
    ROS_FATAL("No stop line at traffic_light_id = %ld, please fix the map!", traffic_light.id());
    return true;
  }

  return registered_task_id_set_.count(tl_stop_line_opt->id()) != 0;
}

void TrafficLightModuleManager::registerTask(const lanelet::TrafficLight& traffic_light) {
  ROS_INFO("Registered Traffic Light Task");

  const auto tl_stop_line_opt = traffic_light.stopLine();

  if (!tl_stop_line_opt) {
    ROS_FATAL("No stop line at traffic_light_id = %ld, please fix the map!", traffic_light.id());
    return;
  }

  registered_task_id_set_.emplace(tl_stop_line_opt->id());
}

void TrafficLightModuleManager::unregisterTask(const lanelet::TrafficLight& traffic_light) {
  ROS_INFO("Unregistered Traffic Light Task");

  const auto tl_stop_line_opt = traffic_light.stopLine();
  registered_task_id_set_.emplace(tl_stop_line_opt->id());
}

}  // namespace behavior_planning

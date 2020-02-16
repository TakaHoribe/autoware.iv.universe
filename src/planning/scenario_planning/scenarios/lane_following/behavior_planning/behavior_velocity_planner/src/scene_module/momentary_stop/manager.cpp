#include <scene_module/momentary_stop/manager.hpp>

namespace behavior_planning {


bool MomentaryStopModuleManager::run(const autoware_planning_msgs::PathWithLaneId& input,
                                    autoware_planning_msgs::PathWithLaneId& output) {
  autoware_planning_msgs::PathWithLaneId input_path = input;
  for (size_t i = 0; i < scene_modules_ptr_.size(); ++i) {
    autoware_planning_msgs::PathWithLaneId output_path;
    if (scene_modules_ptr_.at(i)->run(input_path, output_path)) input_path = output_path;
  }
  debuger.publish();
  output = input_path;
  return true;
}

bool MomentaryStopModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                                std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  geometry_msgs::PoseStamped self_pose = planner_data_->current_pose;
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  for (size_t i = 0; i < input.points.size(); ++i) {
    for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j) {
      std::vector<std::shared_ptr<const lanelet::TrafficSign>> traffic_sign_reg_elems =
          (lanelet_map_ptr->laneletLayer.get(input.points.at(i).lane_ids.at(j)))
              .regulatoryElementsAs<const lanelet::TrafficSign>();
      if (traffic_sign_reg_elems.empty()) continue;
      std::shared_ptr<const lanelet::TrafficSign> traffic_sign = traffic_sign_reg_elems.front();
      if (traffic_sign->type() == "stop_sign") {
        lanelet::ConstLineStrings3d traffic_sign_stoplines = traffic_sign->refLines();
        for (const auto& traffic_sign_stopline : traffic_sign_stoplines) {
          if (!isRunning(traffic_sign_stopline)) {
            v_module_ptr.push_back(
                std::make_shared<MomentaryStopModule>(this, traffic_sign_stopline, input.points.at(i).lane_ids.at(j)));
          }
        }
      }
    }
  }
  return true;
}

bool MomentaryStopModuleManager::isRunning(const lanelet::ConstLineString3d& stop_line) {
  if (task_id_direct_map_.count(stop_line) == 0) return false;
  return true;
}

bool MomentaryStopModuleManager::registerTask(const lanelet::ConstLineString3d& stop_line,
                                              const boost::uuids::uuid& uuid) {
  ROS_INFO("Registered Momentary Stop Task");
  task_id_direct_map_.emplace(stop_line, boost::lexical_cast<std::string>(uuid));
  task_id_reverse_map_.emplace(boost::lexical_cast<std::string>(uuid), stop_line);
  return true;
}
bool MomentaryStopModuleManager::unregisterTask(const boost::uuids::uuid& uuid) {
  ROS_INFO("Unregistered Momentary Stop Task");
  task_id_direct_map_.erase(task_id_reverse_map_.at(boost::lexical_cast<std::string>(uuid)));
  task_id_reverse_map_.erase(boost::lexical_cast<std::string>(uuid));
  return true;
}

}  // namespace behavior_planning

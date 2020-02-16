#include <scene_module/traffic_light/manager.hpp>

#include <map>

#include <tf2/utils.h>

namespace behavior_planning {

TrafficLightModuleManager::TrafficLightModuleManager() : nh_(""), pnh_("~") {}

bool TrafficLightModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                                               std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) {
  geometry_msgs::PoseStamped self_pose = *planner_data_->current_pose;
  const auto lanelet_map_ptr = planner_data_->lanelet_map;
  const auto routing_graph_ptr = planner_data_->routing_graph;

  for (const auto& point : input.points) {
    for (const auto& lane_id : point.lane_ids) {
      std::vector<std::shared_ptr<const lanelet::TrafficLight>> tl_reg_elems =
          (lanelet_map_ptr->laneletLayer.get(lane_id)).regulatoryElementsAs<const lanelet::TrafficLight>();
      for (const auto& tl_reg_elem : tl_reg_elems) {
        if (!isRunning(*tl_reg_elem)) {
          v_module_ptr.push_back(std::make_shared<TrafficLightModule>(this, tl_reg_elem, lane_id));
        }
      }
    }
  }

  return true;
}

bool TrafficLightModuleManager::run(const autoware_planning_msgs::PathWithLaneId& input,
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

bool TrafficLightModuleManager::isRunning(const lanelet::TrafficLight& traffic_light) {
  lanelet::ConstLineString3d tl_stop_line;
  lanelet::Optional<lanelet::ConstLineString3d> tl_stopline_opt = traffic_light.stopLine();
  if (!!tl_stopline_opt)
    tl_stop_line = tl_stopline_opt.get();
  else {
    ROS_ERROR("cannot get traffic light stop line. This is dangerous. f**k lanelet2. (line: %d)", __LINE__);
    return true;
  }
  // lanelet::ConstLineString3d tl_stop_line = *(traffic_light.stopLine());
  if (task_id_direct_map_.count(tl_stop_line) == 0) return false;
  return true;
}

bool TrafficLightModuleManager::registerTask(const lanelet::TrafficLight& traffic_light,
                                             const boost::uuids::uuid& uuid) {
  ROS_INFO("Registered Traffic Light Task");
  lanelet::ConstLineString3d tl_stop_line;
  lanelet::Optional<lanelet::ConstLineString3d> tl_stopline_opt = traffic_light.stopLine();
  if (!!tl_stopline_opt)
    tl_stop_line = tl_stopline_opt.get();
  else {
    ROS_ERROR("cannot get traffic light stop line. This is dangerous. f**k lanelet2. (line: %d)", __LINE__);
    return false;
  }
  // const lanelet::ConstLineString3d tl_stop_line = *(traffic_light.stopLine());
  task_id_direct_map_.emplace(tl_stop_line, boost::lexical_cast<std::string>(uuid));
  task_id_reverse_map_.emplace(boost::lexical_cast<std::string>(uuid), tl_stop_line);
  return true;
}
bool TrafficLightModuleManager::unregisterTask(const boost::uuids::uuid& uuid) {
  ROS_INFO("Unregistered Traffic Light Task");
  task_id_direct_map_.erase(task_id_reverse_map_.at(boost::lexical_cast<std::string>(uuid)));
  task_id_reverse_map_.erase(boost::lexical_cast<std::string>(uuid));
  return true;
}

}  // namespace behavior_planning

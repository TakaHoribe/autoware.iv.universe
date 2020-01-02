#include <scene_module/traffic_light/traffic_light.hpp>
#include <behavior_velocity_planner/api.hpp>
#include <map>

namespace behavior_planning
{
TrafficLightModule::TrafficLightModule(TrafficLightModuleManager *manager_ptr,
                                       const std::shared_ptr<lanelet::TrafficLight const> traffic_light_ptr,
                                       const int lane_id)
    : manager_ptr_(manager_ptr),
      traffic_light_ptr_(traffic_light_ptr),
      lane_id_(lane_id),
      stop_margin_(0.0),
      task_id_(boost::uuids::random_generator()())

{
        // if (manager_ptr_ != nullptr)
        // manager_ptr_->registerTask(*traffic_light_ptr, task_id_);
}

bool TrafficLightModule::run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output)
{
    return true;
}
bool TrafficLightModule::endOfLife(const autoware_planning_msgs::PathWithLaneId &input)
{
    return true;
}

bool TrafficLightModule::getBackwordPointFromBasePoint(const Eigen::Vector2d &line_point1,
                                                       const Eigen::Vector2d &line_point2,
                                                       const Eigen::Vector2d &base_point,
                                                       const double backward_length,
                                                       Eigen::Vector2d &output_point)
{
    Eigen::Vector2d line_vec = line_point2 - line_point1;
    Eigen::Vector2d backward_vec = backward_length * line_vec.normalized();
    output_point = base_point + backward_vec;
    return true;
}

TrafficLightModuleManager::TrafficLightModuleManager() : nh_(""), pnh_("~")
{
}

bool TrafficLightModuleManager::startCondition(const autoware_planning_msgs::PathWithLaneId &input,
                                               std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr)
{
    geometry_msgs::PoseStamped self_pose;
    if (!getCurrentSelfPose(self_pose))
        return false;
    lanelet::LaneletMapConstPtr lanelet_map_ptr;
    lanelet::routing::RoutingGraphConstPtr routing_graph_ptr;
    if (!getLaneletMap(lanelet_map_ptr, routing_graph_ptr))
        return false;

#if 0
    std::map<int, lanelet::ConstLanelet> lanelet_map;
    for (size_t i = 0; i < input.points.size(); ++i)
    {
        for (size_t j = 0; j < input.points.at(i).lane_ids.size(); ++j)
        {
            lanelet_map[input.points.at(i).lane_ids.at(j)] = lanelet_map_ptr->laneletLayer.get(input.points.at(i).lane_ids.at(j));
        }
    }

    lanelet::ConstLanelets route_lanelets;
    for (auto itr = lanelet_map.begin(); itr != lanelet_map.end(); ++itr)
    {
        route_lanelets.push_back(itr->second);
    }
    std::vector<lanelet::AutowareTrafficLightConstPtr> route_lanelet_traffic_lights = lanelet::utils::query::autowareTrafficLights(route_lanelets);
    for (auto tl_itr = route_lanelet_traffic_lights.begin(); tl_itr != route_lanelet_traffic_lights.end(); ++tl_itr)
    {
        lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

        auto lights = tl->trafficLights();
        for (auto lsp : lights)
        {
            if (!lsp.isLineString()) // traffic ligths must be linestrings
                continue;
            // static_cast<lanelet::ConstLineString3d>(lsp);
        }
    }
#endif

    for (const auto &point : input.points)
    {
        for (const auto &lane_id : point.lane_ids)
        {
            std::vector<std::shared_ptr<const lanelet::TrafficLight>> tl_reg_elems =
                (lanelet_map_ptr->laneletLayer.get(lane_id)).regulatoryElementsAs<const lanelet::TrafficLight>();
            // for (const auto &tl_reg_elem: tl_reg_elems)
            // std::shared_ptr<const lanelet::TrafficLight> tl_reg_elem = traffic_light_reg_elems.front();
            for (const auto &tl_reg_elem : tl_reg_elems)
            {
                // for (const auto &tl : tl_reg_elem->trafficLights())
                // {
                // lanelet::ConstLineStringOrPolygon3d theLight = tl_reg_elem->trafficLights().front();
                // }
                if(!isRunning(*tl_reg_elem)){
                    v_module_ptr.push_back(std::make_shared<TrafficLightModule>(this, tl_reg_elem, lane_id));
                }
                lanelet::ConstLineString3d stop_line = *(tl_reg_elem->stopLine());
            }
        }
    }

    return true;
}
bool TrafficLightModuleManager::isRunning(const lanelet::TrafficLight &traffic_light)
{
    const lanelet::ConstLineString3d &tl_stop_line = *(traffic_light.stopLine());
    if (task_id_direct_map_.count(tl_stop_line) == 0)
        return false;
    return true;
}

bool TrafficLightModuleManager::registerTask(const lanelet::TrafficLight &traffic_light, const boost::uuids::uuid &uuid)
{
    ROS_INFO("Registered Traffic Light Task");
    const lanelet::ConstLineString3d &tl_stop_line = *(traffic_light.stopLine());
    task_id_direct_map_.emplace(tl_stop_line, boost::lexical_cast<std::string>(uuid));
    task_id_reverse_map_.emplace(boost::lexical_cast<std::string>(uuid), tl_stop_line);
    return true;
}
bool TrafficLightModuleManager::unregisterTask(const boost::uuids::uuid &uuid)
{
    ROS_INFO("Unregistered Traffic Light Task");
    task_id_direct_map_.erase(task_id_reverse_map_.at(boost::lexical_cast<std::string>(uuid)));
    task_id_reverse_map_.erase(boost::lexical_cast<std::string>(uuid));
    return true;
}


} // namespace behavior_planning
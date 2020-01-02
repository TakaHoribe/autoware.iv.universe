#pragma once
#include <ros/ros.h>
#include <scene_module/scene_module_interface.hpp>
#include <unordered_map>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <string>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_extension/utility/query.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace behavior_planning
{
class TrafficLightModuleManager;

class TrafficLightModule : public SceneModuleInterface
{
public:
public:
    TrafficLightModule(TrafficLightModuleManager* manager_ptr,
                        const std::shared_ptr<lanelet::TrafficLight const> traffic_light_ptr,
                        const int lane_id);
    bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output) override;
    bool endOfLife(const autoware_planning_msgs::PathWithLaneId &input) override;
    ~TrafficLightModule(){};

private:
    bool getBackwordPointFromBasePoint(const Eigen::Vector2d &line_point1,
                                       const Eigen::Vector2d &line_point2,
                                       const Eigen::Vector2d &base_point,
                                       const double backward_length,
                                       Eigen::Vector2d &output_point);
    TrafficLightModuleManager* manager_ptr_;
    std::shared_ptr<lanelet::TrafficLight const> traffic_light_ptr_;
    int lane_id_;
    double stop_margin_;
    boost::uuids::uuid task_id_;

};

class TrafficLightModuleManager : public SceneModuleManagerInterface
{
public:
    TrafficLightModuleManager();
    ~TrafficLightModuleManager(){};
    bool startCondition(const autoware_planning_msgs::PathWithLaneId &input, std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr) override;
    bool isRunning(const lanelet::TrafficLight &traffic_light);
    bool registerTask(const lanelet::TrafficLight &traffic_light, const boost::uuids::uuid &uuid);
    bool unregisterTask(const boost::uuids::uuid &uuid);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unordered_map<lanelet::ConstLineString3d, std::string> task_id_direct_map_;
    std::unordered_map<std::string, lanelet::ConstLineString3d> task_id_reverse_map_;

};


} // namespace behavior_planning
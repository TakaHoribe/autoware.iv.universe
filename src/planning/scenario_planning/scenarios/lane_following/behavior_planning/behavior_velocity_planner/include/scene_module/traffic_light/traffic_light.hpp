#pragma once
#include <ros/ros.h>
#include <scene_module/scene_module_interface.hpp>
#include <scene_module/traffic_light/debug_marker.hpp>
#include <unordered_map>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
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
    ~TrafficLightModule();

private:
    bool getBackwordPointFromBasePoint(const Eigen::Vector2d &line_point1,
                                       const Eigen::Vector2d &line_point2,
                                       const Eigen::Vector2d &base_point,
                                       const double backward_length,
                                       Eigen::Vector2d &output_point);
    bool insertTargetVelocityPoint(const autoware_planning_msgs::PathWithLaneId &input,
                                   const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &stop_line,
                                   const double &margin,
                                   const double &velocity,
                                   autoware_planning_msgs::PathWithLaneId &output);
    bool getHighestConfidenceTrafficLightState(lanelet::ConstLineStringsOrPolygons3d &traffic_lights,
                                               autoware_traffic_light_msgs::TrafficLightState &highest_confidence_tl_state);
    bool createTargetPoint(const autoware_planning_msgs::PathWithLaneId &input,
                          const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>> &stop_line,
                          const double &margin,
                          size_t &target_point_idx,
                          Eigen::Vector2d &target_point);
    enum class State
    {
        APPROARCH,
        GO_OUT
    };
    TrafficLightModuleManager *manager_ptr_;
    State state_;
    std::shared_ptr<lanelet::TrafficLight const> traffic_light_ptr_;
    int lane_id_;
    double stop_margin_;
    double tl_state_timeout_;
    double max_stop_acceleration_threshold_;
    boost::uuids::uuid task_id_;

};

class TrafficLightModuleManager : public SceneModuleManagerInterface
{
public:
    TrafficLightModuleManager();
    ~TrafficLightModuleManager(){};
    bool startCondition(const autoware_planning_msgs::PathWithLaneId &input, std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr) override;
    bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output) override;
    bool isRunning(const lanelet::TrafficLight &traffic_light);
    bool registerTask(const lanelet::TrafficLight &traffic_light, const boost::uuids::uuid &uuid);
    bool unregisterTask(const boost::uuids::uuid &uuid);
    TrafficLightDebugMarkersManager debuger;

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unordered_map<lanelet::ConstLineString3d, std::string> task_id_direct_map_;
    std::unordered_map<std::string, lanelet::ConstLineString3d> task_id_reverse_map_;

};


} // namespace behavior_planning
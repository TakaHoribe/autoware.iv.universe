#pragma once

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>

#include <scene_module/scene_module_interface.hpp>
#include <unordered_map>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign/list_of.hpp>
#include <string>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/utility/utilities.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace behavior_planning
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;

class RightTurnModuleManager;
class RightTurnModuleDebugger;

/*
 * ========================= Right Turn Module =========================
 */
class RightTurnModule : public SceneModuleInterface
{
public:
    RightTurnModule(const int lane_id, RightTurnModuleManager *right_turn_module_manager);
    ~RightTurnModule(){};
    bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output) override;
    bool endOfLife(const autoware_planning_msgs::PathWithLaneId &input) override;

private:
    const int assigned_lane_id_;
    RightTurnModuleManager *right_turn_module_manager_;

    bool setStopVelocityFrom(const size_t stop_point_id, autoware_planning_msgs::PathWithLaneId &input);
    bool getObjectiveLane();
    Polygon convertToBoostGeometryPolygon(const lanelet::ConstLanelet &lanelet);
    bool checkDynamicCollision(const autoware_planning_msgs::PathWithLaneId &path, const autoware_perception_msgs::DynamicObject &object);

};

/*
 * ========================= Right Turn Module Debugger =========================
 */
class RightTurnModuleDebugger
{
public:
    RightTurnModuleDebugger();
    ~RightTurnModuleDebugger(){};

    void publishLaneletsArea(const std::vector<lanelet::ConstLanelet> &lanelets, const std::string &ns);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher debug_viz_pub_;
};

/*
 * ========================= Right Turn Module Manager =========================
 */
class RightTurnModuleManager : public SceneModuleManagerInterface
{
public:
    RightTurnModuleManager(){};
    ~RightTurnModuleManager(){};
    bool startCondition(const autoware_planning_msgs::PathWithLaneId &input, std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr) override;
    RightTurnModuleDebugger debugger_;
    void unregisterTask(const int lane_id);

private:
    std::vector<int> registered_lane_ids_;
    
    bool isRunning(const int lane_id);
    void registerTask(const int lane_id);

};

} // namespace behavior_planning
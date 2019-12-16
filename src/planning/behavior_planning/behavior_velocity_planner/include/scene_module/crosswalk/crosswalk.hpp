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
#include <lanelet2_routing/RoutingGraphContainer.h>

namespace behavior_planning
{
class CrosswalkModuleManager;

class CrosswalkModule : public SceneModuleInterface
{
public:
    CrosswalkModule(CrosswalkModuleManager* manager_ptr,
                        const lanelet::ConstLanelet &crosswalk,
                        const int lane_id);
    bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output) override;
    bool endOfLife(const autoware_planning_msgs::PathWithLaneId &input) override;
    ~CrosswalkModule(){};

private:
    enum class State
    {
        APPROARCH,
        STOP,
        START
    };
    CrosswalkModuleManager* manager_ptr_;
    State state_;
    int lane_id_;
    lanelet::ConstLanelet crosswalk_;
    double stop_margin_;
    boost::uuids::uuid task_id_;
};

class CrosswalkDebugMarkersManager
{
public:
    CrosswalkDebugMarkersManager();
    ~CrosswalkDebugMarkersManager(){};
    void pushCollisionLine(const std::vector<Eigen::Vector3d> &line);
    void pushCollisionLine(const std::vector<Eigen::Vector2d> &line);
    void pushCollisionPoint(const Eigen::Vector3d &point);
    void pushCollisionPoint(const Eigen::Vector2d &point);
    void pushStopPoint(const Eigen::Vector3d &point);
    void pushStopPoint(const Eigen::Vector2d &point);
    void pushCrosswalkPolygon(const std::vector<Eigen::Vector3d> &polygon);
    void pushCrosswalkPolygon(const std::vector<Eigen::Vector2d> &polygon);

    void publish();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher debug_viz_pub_;
    std::vector<Eigen::Vector3d> collision_points_;
    std::vector<Eigen::Vector3d> stop_points_;
    std::vector<std::vector<Eigen::Vector3d>> collision_lines_;
    std::vector<std::vector<Eigen::Vector3d>> crosswalk_polygons_;
};

class CrosswalkModuleManager : public SceneModuleManagerInterface
{
public:
    CrosswalkModuleManager();
    ~CrosswalkModuleManager(){};
    bool startCondition(const autoware_planning_msgs::PathWithLaneId &input, std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr) override;
    bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output) override;
    bool isRunning(const lanelet::ConstLanelet &crosswalk);
    bool registerTask(const lanelet::ConstLanelet &crosswalk, const boost::uuids::uuid &uuid);
    bool unregisterTask(const boost::uuids::uuid &uuid);
    CrosswalkDebugMarkersManager debuger;
    
private:
    std::unordered_map<lanelet::ConstLanelet, std::string> task_id_direct_map_;
    std::unordered_map<std::string, lanelet::ConstLanelet> task_id_reverse_map_;
    std::shared_ptr<lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;
};

} // namespace behavior_planning
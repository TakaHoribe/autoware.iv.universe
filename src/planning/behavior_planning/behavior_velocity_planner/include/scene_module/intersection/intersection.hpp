#pragma once
#include <string>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/assign/list_of.hpp>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_extension/utility/utilities.h>

#include <scene_module/scene_module_interface.hpp>

namespace behavior_planning
{
namespace bg = boost::geometry;
using Point = bg::model::d2::point_xy<double>;
using Polygon = bg::model::polygon<Point, false>;

class IntersectionModuleManager;
class IntersectionModuleDebugger;

/*
 * ========================= Intersection Module =========================
 */
class IntersectionModule : public SceneModuleInterface
{
public:
    IntersectionModule(const int lane_id, IntersectionModuleManager *intersection_module_manager);
    ~IntersectionModule(){};

    /**
     * @brief plan go-stop velocity at traffic crossing with collision check between reference path and object predicted path
     */
    bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output) override;

    /**
     * @brief kill instance if there is no assigned_lane_id in input path
     */
    bool endOfLife(const autoware_planning_msgs::PathWithLaneId &input) override;

private:
    const int assigned_lane_id_;                        //< @brief object lane id (unique for this instance)
    int stop_line_idx_;                                 //< @brief stop-line index
    int judge_line_idx_;                                //< @brief stop-judgement-line index
    double judge_line_dist_;                            //< @brief distance from stop-line to stop-judgement line
    double approaching_speed_to_stopline_;              //< @brief speed when approaching stop-line (should be slow)
    double path_expand_width_;                          //< @brief path width to calculate the edge line for both side
    IntersectionModuleManager *intersection_module_manager_; //< @brief manager pointer
    bool show_debug_info_;

    /**
     * @brief set velocity from idx to the end point
     */
    bool setVelocityFrom(const size_t idx, const double vel, autoware_planning_msgs::PathWithLaneId &input);

    /**
     * @brief get objective lanelets for detection area
     */
    bool getObjectiveLanelets(lanelet::LaneletMapConstPtr lanelet_map_ptr, lanelet::routing::RoutingGraphConstPtr routing_graph_ptr, 
                              const int lane_id, std::vector<lanelet::ConstLanelet> &objective_lanelets);

    /**
     * @brief check collision with path & dynamic object predicted path
     */
    bool checkPathCollision(const autoware_planning_msgs::PathWithLaneId &path, const autoware_perception_msgs::DynamicObject &object);

    /**
     * @brief check collision for all lanelet area & dynamic objects (call checkPathCollision() as actual collision check algorithm inside this function)
     */
    bool checkCollision(const autoware_planning_msgs::PathWithLaneId &path, const std::vector<lanelet::ConstLanelet> &objective_lanelets,
                        const std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> objects_ptr, 
                        const double path_width, bool &is_collision);
                        
    /**
     * @brief calculate right and left path edge line
     */
    bool generateEdgeLine(const autoware_planning_msgs::PathWithLaneId &path, const double path_width,
                          autoware_planning_msgs::PathWithLaneId &path_r, autoware_planning_msgs::PathWithLaneId &path_l);
    /**
     * @brief set stop-line and stop-judgement-line index. This may modificates path size due to interpolate insertion.
     */
    bool setStopLineIdx(const double judge_line_dist, autoware_planning_msgs::PathWithLaneId &path, int &stop_line_idx, int &judge_line_idx);

    /**
     * @brief convert from lanelet to boost polygon
     */
    Polygon convertToBoostGeometryPolygon(const lanelet::ConstLanelet &lanelet);

    enum class State
    {
        STOP = 0,
        GO,
    };

    /**
     * @brief Manage stop-go states with safety margin time.
     */
    class StateMachine
    {
    public:
        StateMachine()
        {
            state_ = IntersectionModule::State::STOP;
            margin_time_ = 0.0;
        };

        /**
         * @brief set request state command with margin time
         */
        void setStateWithMarginTime(IntersectionModule::State state);

        /**
         * @brief set request state command directly
         */
        void setState(IntersectionModule::State state);

        /**
         * @brief set margin time
         */
        void setMarginTime(const double t);

        /**
         * @brief get current state
         */
        IntersectionModule::State getState();

    private:
        State state_;                           //< @brief current state
        double margin_time_;                    //< @brief margin time when transit to Go from Stop
        std::shared_ptr<ros::Time> start_time_; //< @brief timer start time when received Go state when current state is Stop
    } state_machine_;                           //< @brief for state management
};

/*
 * ========================= Intersection Module Debugger =========================
 */
class IntersectionModuleDebugger
{
public:
    IntersectionModuleDebugger();
    ~IntersectionModuleDebugger(){};

    void publishLaneletsArea(const std::vector<lanelet::ConstLanelet> &lanelets, const std::string &ns);
    void publishPath(const autoware_planning_msgs::PathWithLaneId &path, const std::string &ns, double r, double g, double b);
    void publishPose(const geometry_msgs::Pose &pose, const std::string &ns, double r, double g, double b);

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher debug_viz_pub_;
};

/*
 * ========================= Intersection Module Manager =========================
 */
class IntersectionModuleManager : public SceneModuleManagerInterface
{
public:
    IntersectionModuleManager(){};
    ~IntersectionModuleManager(){};
    bool startCondition(const autoware_planning_msgs::PathWithLaneId &input, std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr) override;
    IntersectionModuleDebugger debugger_;
    void unregisterTask(const int lane_id);

private:
    std::vector<int> registered_lane_ids_;

    bool isRunning(const int lane_id);
    void registerTask(const int lane_id);
};

} // namespace behavior_planning
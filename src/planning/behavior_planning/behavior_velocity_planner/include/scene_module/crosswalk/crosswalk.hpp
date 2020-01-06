#pragma once

#include <ros/ros.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>
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
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

namespace behavior_planning
{
class CrosswalkModuleManager;

class CrosswalkModule : public SceneModuleInterface
{
public:
    CrosswalkModule(CrosswalkModuleManager *manager_ptr,
                    const lanelet::ConstLanelet &crosswalk,
                    const int lane_id);
    bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output) override;
    bool endOfLife(const autoware_planning_msgs::PathWithLaneId &input) override;
    ~CrosswalkModule(){};

private:
    bool getBackwordPointFromBasePoint(const Eigen::Vector2d &line_point1,
                                       const Eigen::Vector2d &line_point2,
                                       const Eigen::Vector2d &base_point,
                                       const double backward_length,
                                       Eigen::Vector2d &output_point);
    bool checkSlowArea(const autoware_planning_msgs::PathWithLaneId &input,
                       const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &polygon,
                       const std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> &objects_ptr,
                       const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &no_ground_pointcloud_ptr,
                       autoware_planning_msgs::PathWithLaneId &output);
    bool checkStopArea(const autoware_planning_msgs::PathWithLaneId &input,
                       const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &polygon,
                       const std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const> &objects_ptr,
                       const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &no_ground_pointcloud_ptr,
                       autoware_planning_msgs::PathWithLaneId &output);
    bool insertTargetVelocityPoint(const autoware_planning_msgs::PathWithLaneId &input,
                                   const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false> &polygon,
                                   const double &margin,
                                   const double &velocity,
                                   autoware_planning_msgs::PathWithLaneId &output);
    enum class State
    {
        APPROARCH,
        INSIDE,
        GO_OUT
    };
    CrosswalkModuleManager *manager_ptr_;
    State state_;
    int lane_id_;
    lanelet::ConstLanelet crosswalk_;
    double stop_margin_;
    double stop_dynamic_object_prediction_time_margin_;
    double slow_margin_;
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
    void pushStopPose(const geometry_msgs::Pose &pose);
    void pushSlowPose(const geometry_msgs::Pose &pose);
    void pushCrosswalkPolygon(const std::vector<Eigen::Vector3d> &polygon);
    void pushCrosswalkPolygon(const std::vector<Eigen::Vector2d> &polygon);
    void pushStopPolygon(const std::vector<Eigen::Vector3d> &polygon);
    void pushStopPolygon(const std::vector<Eigen::Vector2d> &polygon);
    void pushSlowPolygon(const std::vector<Eigen::Vector3d> &polygon);
    void pushSlowPolygon(const std::vector<Eigen::Vector2d> &polygon);

    void publish();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher debug_viz_pub_;
    std::vector<Eigen::Vector3d> collision_points_;
    std::vector<geometry_msgs::Pose> stop_poses_;
    std::vector<geometry_msgs::Pose> slow_poses_;
    std::vector<std::vector<Eigen::Vector3d>> collision_lines_;
    std::vector<std::vector<Eigen::Vector3d>> crosswalk_polygons_;
    std::vector<std::vector<Eigen::Vector3d>> stop_polygons_;
    std::vector<std::vector<Eigen::Vector3d>> slow_polygons_;
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
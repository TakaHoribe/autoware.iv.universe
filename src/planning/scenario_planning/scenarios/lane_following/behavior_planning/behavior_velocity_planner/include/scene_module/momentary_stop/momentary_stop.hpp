#pragma once

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <scene_module/scene_module_interface.hpp>
#include <scene_module/momentary_stop/debug_marker.hpp>
#include <string>
#include <unordered_map>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace behavior_planning {
class MomentaryStopModuleManager;

class MomentaryStopModule : public SceneModuleInterface {
 public:
  MomentaryStopModule(MomentaryStopModuleManager* manager_ptr, const lanelet::ConstLineString3d& stop_line,
                      const int lane_id);
  bool run(const autoware_planning_msgs::PathWithLaneId& input,
           autoware_planning_msgs::PathWithLaneId& output) override;
  bool endOfLife(const autoware_planning_msgs::PathWithLaneId& input) override;
  ~MomentaryStopModule();

 private:
  bool getBackwordPointFromBasePoint(const Eigen::Vector2d& line_point1, const Eigen::Vector2d& line_point2,
                                     const Eigen::Vector2d& base_point, const double backward_length,
                                     Eigen::Vector2d& output_point);

  enum class State { APPROARCH, STOP, START };
  MomentaryStopModuleManager* manager_ptr_;
  State state_;
  int lane_id_;
  lanelet::ConstLineString3d stop_line_;
  double stop_margin_;
  boost::uuids::uuid task_id_;
};

class MomentaryStopModuleManager : public SceneModuleManagerInterface {
 public:
  MomentaryStopModuleManager(){};
  ~MomentaryStopModuleManager(){};
  bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                      std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) override;
  bool run(const autoware_planning_msgs::PathWithLaneId& input,
           autoware_planning_msgs::PathWithLaneId& output) override;
  bool isRunning(const lanelet::ConstLineString3d& stop_line);
  bool registerTask(const lanelet::ConstLineString3d& stop_line, const boost::uuids::uuid& uuid);
  bool unregisterTask(const boost::uuids::uuid& uuid);
  MomentaryStopDebugMarkersManager debuger;

 private:
  std::unordered_map<lanelet::ConstLineString3d, std::string> task_id_direct_map_;
  std::unordered_map<std::string, lanelet::ConstLineString3d> task_id_reverse_map_;
};

}  // namespace behavior_planning
#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/momentary_stop/debug.h>
#include <scene_module/momentary_stop/scene.h>
#include <scene_module/scene_module_interface.h>

namespace behavior_planning {

class MomentaryStopModuleManager : public SceneModuleManagerInterface {
 public:
  bool run(const autoware_planning_msgs::PathWithLaneId& input,
           autoware_planning_msgs::PathWithLaneId& output) override;
  bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                      std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) override;

  bool isRegistered(const lanelet::ConstLineString3d& stop_line);
  void registerTask(const lanelet::ConstLineString3d& stop_line);
  void unregisterTask(const lanelet::ConstLineString3d& stop_line);

  MomentaryStopDebugMarkersManager debuger;

 private:
  std::set<int64_t> registered_task_id_set_;
};

}  // namespace behavior_planning

#pragma once

#include <string>
#include <unordered_map>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/lexical_cast.hpp>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_generators.hpp>
#include <boost/uuid/uuid_io.hpp>

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
  bool isRunning(const lanelet::ConstLineString3d& stop_line);
  bool registerTask(const lanelet::ConstLineString3d& stop_line, const boost::uuids::uuid& uuid);
  bool unregisterTask(const boost::uuids::uuid& uuid);

  MomentaryStopDebugMarkersManager debuger;

 private:
  std::unordered_map<lanelet::ConstLineString3d, std::string> task_id_direct_map_;
  std::unordered_map<std::string, lanelet::ConstLineString3d> task_id_reverse_map_;
};

}  // namespace behavior_planning

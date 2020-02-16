#pragma once

#include <string>
#include <unordered_map>
#include <vector>

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

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <scene_module/crosswalk/debug.hpp>
#include <scene_module/crosswalk/scene.hpp>
#include <scene_module/scene_module_interface.hpp>

namespace behavior_planning {

class CrosswalkModuleManager : public SceneModuleManagerInterface {
 public:
  CrosswalkModuleManager();

  bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                      std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) override;
  bool run(const autoware_planning_msgs::PathWithLaneId& input,
           autoware_planning_msgs::PathWithLaneId& output) override;
  bool isRunning(const lanelet::ConstLanelet& crosswalk);
  bool registerTask(const lanelet::ConstLanelet& crosswalk, const boost::uuids::uuid& uuid);
  bool unregisterTask(const boost::uuids::uuid& uuid);
  CrosswalkDebugMarkersManager debuger;

 private:
  std::unordered_map<lanelet::ConstLanelet, std::string> task_id_direct_map_;
  std::unordered_map<std::string, lanelet::ConstLanelet> task_id_reverse_map_;
  std::shared_ptr<lanelet::routing::RoutingGraphContainer> overall_graphs_ptr_;
};

}  // namespace behavior_planning

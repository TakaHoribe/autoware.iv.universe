#pragma once

#include <set>
#include <string>
#include <vector>
#include <memory>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/lexical_cast.hpp>

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

#include <scene_module/crosswalk/debug.h>
#include <scene_module/crosswalk/scene.h>
#include <scene_module/scene_module_interface.h>

namespace behavior_planning {

class CrosswalkModuleManager : public SceneModuleManagerInterface {
 public:
  bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                      std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) override;

  void debug() override { debugger_.publish(); }
  CrosswalkDebugMarkersManager debugger_;  // TODO: remove

  bool isRegistered(const lanelet::ConstLanelet& crosswalk);
  void registerTask(const lanelet::ConstLanelet& crosswalk);
  void unregisterTask(const lanelet::ConstLanelet& crosswalk);

 private:
  std::set<int64_t> registered_task_id_set_;
};

}  // namespace behavior_planning

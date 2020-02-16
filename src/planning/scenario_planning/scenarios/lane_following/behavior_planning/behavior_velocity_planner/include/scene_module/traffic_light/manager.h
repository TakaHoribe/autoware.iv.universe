#pragma once

#include <string>
#include <unordered_map>

#include <boost/assert.hpp>
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

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>
#include <scene_module/traffic_light/debug.h>
#include <scene_module/traffic_light/scene.h>

namespace behavior_planning {

class TrafficLightModuleManager : public SceneModuleManagerInterface {
 public:
  TrafficLightModuleManager();

  bool startCondition(const autoware_planning_msgs::PathWithLaneId& input,
                      std::vector<std::shared_ptr<SceneModuleInterface>>& v_module_ptr) override;
  bool run(const autoware_planning_msgs::PathWithLaneId& input,
           autoware_planning_msgs::PathWithLaneId& output) override;
  bool isRunning(const lanelet::TrafficLight& traffic_light);
  bool registerTask(const lanelet::TrafficLight& traffic_light, const boost::uuids::uuid& uuid);
  bool unregisterTask(const boost::uuids::uuid& uuid);
  TrafficLightDebugMarkersManager debuger;

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  std::unordered_map<lanelet::ConstLineString3d, std::string> task_id_direct_map_;
  std::unordered_map<std::string, lanelet::ConstLineString3d> task_id_reverse_map_;
};

}  // namespace behavior_planning

#pragma once

#include <string>
#include <unordered_map>

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

#include <scene_module/scene_module_interface.h>

namespace behavior_planning {
class MomentaryStopModuleManager;

class MomentaryStopModule : public SceneModuleInterface {
 public:
  MomentaryStopModule(MomentaryStopModuleManager* manager_ptr, const lanelet::ConstLineString3d& stop_line,
                      const int lane_id);

  bool run(const autoware_planning_msgs::PathWithLaneId& input,
           autoware_planning_msgs::PathWithLaneId& output) override;
  bool endOfLife(const autoware_planning_msgs::PathWithLaneId& input) override;

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
};

}  // namespace behavior_planning

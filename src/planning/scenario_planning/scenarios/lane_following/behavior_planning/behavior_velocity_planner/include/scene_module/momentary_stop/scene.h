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

class MomentaryStopModule : public SceneModuleInterface {
 public:
  explicit MomentaryStopModule(const int64_t module_id, const lanelet::ConstLineString3d& stop_line);

  bool modifyPathVelocity(autoware_planning_msgs::PathWithLaneId* path) override;

 private:
  bool getBackwordPointFromBasePoint(const Eigen::Vector2d& line_point1, const Eigen::Vector2d& line_point2,
                                     const Eigen::Vector2d& base_point, const double backward_length,
                                     Eigen::Vector2d& output_point);

  enum class State { APPROARCH, STOP, START };

  lanelet::ConstLineString3d stop_line_;
  State state_;

  // Paramter
  const double stop_margin_ = 0.0;
};

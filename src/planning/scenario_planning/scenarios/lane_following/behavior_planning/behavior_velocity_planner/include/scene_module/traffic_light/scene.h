#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <boost/assert.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>
#include <scene_module/traffic_light/debug.h>

class TrafficLightModule : public SceneModuleInterface {
 public:
  explicit TrafficLightModule(const int64_t module_id, const lanelet::TrafficLight& traffic_light_reg_elem);

  bool modifyPathVelocity(autoware_planning_msgs::PathWithLaneId* path) override;

 private:
  bool getBackwordPointFromBasePoint(const Eigen::Vector2d& line_point1, const Eigen::Vector2d& line_point2,
                                     const Eigen::Vector2d& base_point, const double backward_length,
                                     Eigen::Vector2d& output_point);

  bool insertTargetVelocityPoint(
      const autoware_planning_msgs::PathWithLaneId& input,
      const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>>& stop_line,
      const double& margin, const double& velocity, autoware_planning_msgs::PathWithLaneId& output);

  bool getHighestConfidenceTrafficLightState(
      lanelet::ConstLineStringsOrPolygons3d& traffic_lights,
      autoware_traffic_light_msgs::TrafficLightState& highest_confidence_tl_state);

  bool createTargetPoint(
      const autoware_planning_msgs::PathWithLaneId& input,
      const boost::geometry::model::linestring<boost::geometry::model::d2::point_xy<double>>& stop_line,
      const double& margin, size_t& target_point_idx, Eigen::Vector2d& target_point);

  // Key Feature
  const lanelet::TrafficLight& traffic_light_reg_elem_;

  // State
  enum class State { APPROARCH, GO_OUT };
  State state_;

  // Parameter
  const double stop_margin_ = 0.0;
  const double tl_state_timeout_ = 1.0;
  const double max_stop_acceleration_threshold_ = -5.0;

  // Debug
};

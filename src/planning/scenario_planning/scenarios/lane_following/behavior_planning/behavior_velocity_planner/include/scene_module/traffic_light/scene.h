#pragma once

#include <memory>
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

namespace behavior_planning {
class TrafficLightModuleManager;

class TrafficLightModule : public SceneModuleInterface {
 public:
 public:
  TrafficLightModule(TrafficLightModuleManager* manager_ptr,
                     const std::shared_ptr<lanelet::TrafficLight const> traffic_light_ptr, const int lane_id);
  ~TrafficLightModule();

  bool run(const autoware_planning_msgs::PathWithLaneId& input,
           autoware_planning_msgs::PathWithLaneId& output) override;
  bool endOfLife(const autoware_planning_msgs::PathWithLaneId& input) override;

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

  enum class State { APPROARCH, GO_OUT };

  TrafficLightModuleManager* manager_ptr_;
  State state_;
  std::shared_ptr<lanelet::TrafficLight const> traffic_light_ptr_;
  int lane_id_;
  double stop_margin_;
  double tl_state_timeout_;
  double max_stop_acceleration_threshold_;
  boost::uuids::uuid task_id_;
};

}  // namespace behavior_planning

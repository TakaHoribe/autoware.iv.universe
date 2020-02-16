#pragma once

#include <memory>
#include <string>

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>

using Point = boost::geometry::model::d2::point_xy<double>;
using Polygon = boost::geometry::model::polygon<Point, false>;

class BlindSpotModule : public SceneModuleInterface {
 public:
  BlindSpotModule(const int64_t module_id, const int64_t lane_id, const std::string& turn_direction);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool modifyPathVelocity(autoware_planning_msgs::PathWithLaneId* path) override;

 private:
  int64_t lane_id_;
  std::string turn_direction_;  //! turn direction : right or left

  int stop_line_idx_;   //! stop-line index
  int judge_line_idx_;  //! stop-judgement-line index

  // Parameter
  const double judge_line_dist_ = 1.5;    //! distance from stop-line to stop-judgement line
  const double path_expand_width_ = 2.0;  //! path width to calculate the edge line for both side
  const bool show_debug_info_ = false;

  /**
   * @brief set velocity from idx to the end point
   */
  bool setVelocityFrom(const size_t idx, const double vel, autoware_planning_msgs::PathWithLaneId& input);

  /**
   * @brief check collision with path & dynamic object predicted path
   */
  bool checkPathCollision(const autoware_planning_msgs::PathWithLaneId& path,
                          const autoware_perception_msgs::DynamicObject& object);

  /**
   * @brief check collision for all lanelet area & dynamic objects (call checkPathCollision() as
   * actual collision check algorithm inside this function)
   */
  bool checkCollision(const autoware_planning_msgs::PathWithLaneId& path,
                      const std::vector<std::vector<geometry_msgs::Point>>& detection_areas,
                      const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr, const double path_width,
                      bool& is_collision);
  /**
   * @brief generates detection area
   */
  bool generateDetectionArea(const geometry_msgs::Pose& current_pose,
                             std::vector<std::vector<geometry_msgs::Point>>& detection_areas);

  /**
   * @brief calculate right and left path edge line
   */
  bool generateEdgeLine(const autoware_planning_msgs::PathWithLaneId& path, const double path_width,
                        autoware_planning_msgs::PathWithLaneId& path_r, autoware_planning_msgs::PathWithLaneId& path_l);
  /**
   * @brief set stop-line and stop-judgement-line index. This may modificates path size due to
   * interpolate insertion.
   */
  bool setStopLineIdx(const int closest, const double judge_line_dist, autoware_planning_msgs::PathWithLaneId& path,
                      int& stop_line_idx, int& judge_line_idx);

  geometry_msgs::Pose getAheadPose(const size_t start_idx, const double ahead_dist,
                                   const autoware_planning_msgs::PathWithLaneId& path) const;
  /**
   * @brief convert from lanelet to boost polygon
   */
  Polygon convertToBoostGeometryPolygon(const std::vector<geometry_msgs::Point>& detection_area);

  enum class State {
    STOP = 0,
    GO,
  };

  /**
   * @brief Manage stop-go states with safety margin time.
   */
  class StateMachine {
   public:
    StateMachine() {
      state_ = BlindSpotModule::State::GO;
      margin_time_ = 0.0;
    }

    /**
     * @brief set request state command with margin time
     */
    void setStateWithMarginTime(BlindSpotModule::State state);

    /**
     * @brief set request state command directly
     */
    void setState(BlindSpotModule::State state);

    /**
     * @brief set margin time
     */
    void setMarginTime(const double t);

    /**
     * @brief get current state
     */
    BlindSpotModule::State getState();

   private:
    State state_;                            //!  current state
    double margin_time_;                     //!  margin time when transit to Go from Stop
    std::shared_ptr<ros::Time> start_time_;  //!  timer start time when received Go state when current state is Stop
  } state_machine_;                          //!  for state management
};

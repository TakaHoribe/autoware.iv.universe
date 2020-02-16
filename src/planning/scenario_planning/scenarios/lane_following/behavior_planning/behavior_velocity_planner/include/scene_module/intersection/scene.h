#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>

namespace behavior_planning {

class IntersectionModuleManager; // To be refactored

class IntersectionModule : public SceneModuleInterface {
 public:
  IntersectionModule(const int lane_id, IntersectionModuleManager* intersection_module_manager);

  /**
   * @brief plan go-stop velocity at traffic crossing with collision check between reference path
   * and object predicted path
   */
  bool run(const autoware_planning_msgs::PathWithLaneId& input,
           autoware_planning_msgs::PathWithLaneId& output) override;

  /**
   * @brief kill instance if there is no assigned_lane_id in input path
   */
  bool endOfLife(const autoware_planning_msgs::PathWithLaneId& input) override;

 private:
  const int assigned_lane_id_;            //< @brief object lane id (unique for this instance)
  int stop_line_idx_;                     //< @brief stop-line index
  int judge_line_idx_;                    //< @brief stop-judgement-line index
  double judge_line_dist_;                //< @brief distance from stop-line to stop-judgement line
  double approaching_speed_to_stopline_;  //< @brief speed when approaching stop-line (should be slow)
  double path_expand_width_;              //< @brief path width to calculate the edge line for both side
  IntersectionModuleManager* intersection_module_manager_;  //< @brief manager pointer
  bool show_debug_info_;
  double baselink_to_front_length_;

  /**
   * @brief set velocity from idx to the end point
   */
  bool setVelocityFrom(const size_t idx, const double vel, autoware_planning_msgs::PathWithLaneId* input);

  /**
   * @brief get objective lanelets for detection area
   */
  bool getObjectiveLanelets(lanelet::LaneletMapConstPtr lanelet_map_ptr,
                            lanelet::routing::RoutingGraphConstPtr routing_graph_ptr, const int lane_id,
                            std::vector<lanelet::ConstLanelet>* objective_lanelets);

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
                      const std::vector<lanelet::ConstLanelet>& objective_lanelets,
                      const autoware_perception_msgs::DynamicObjectArray::ConstPtr objects_ptr,
                      const double path_width);

  /**
   * @brief calculate right and left path edge line
   */
  bool generateEdgeLine(const autoware_planning_msgs::PathWithLaneId& path, const double path_width,
                        autoware_planning_msgs::PathWithLaneId* path_r, autoware_planning_msgs::PathWithLaneId* path_l);

  /**
   * @brief set stop-line and stop-judgement-line index. This may modificates path size due to
   * interpolate insertion.
   */
  bool setStopLineIdx(const int closest, const double judge_line_dist, autoware_planning_msgs::PathWithLaneId* path,
                      int* stop_line_idx, int* judge_line_idx);

  geometry_msgs::Pose getAheadPose(const size_t start_idx, const double ahead_dist,
                                   const autoware_planning_msgs::PathWithLaneId& path) const;

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
      state_ = IntersectionModule::State::GO;
      margin_time_ = 0.0;
    }

    /**
     * @brief set request state command with margin time
     */
    void setStateWithMarginTime(IntersectionModule::State state);

    /**
     * @brief set request state command directly
     */
    void setState(IntersectionModule::State state);

    /**
     * @brief set margin time
     */
    void setMarginTime(const double t);

    /**
     * @brief get current state
     */
    IntersectionModule::State getState();

   private:
    State state_;         //< @brief current state
    double margin_time_;  //< @brief margin time when transit to Go from Stop
    std::shared_ptr<ros::Time>
        start_time_;  //< @brief timer start time when received Go state when current state is Stop
  } state_machine_;   //< @brief for state management
};

}  // namespace behavior_planning

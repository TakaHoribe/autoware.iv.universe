#pragma once

#include <ros/ros.h>

#include <autoware_vehicle_msgs/TurnSignal.h>
#include <turn_signal_decider/data_manager.h>
#include <turn_signal_decider/frenet_coordinate.h>

namespace turn_signal_decider
{
struct TurnSignalParameters
{
  double lane_change_search_distance;  // TODO: change this to time based threshold
  double intersection_search_distance;
};

class TurnSignalDecider
{
private:
  // ROS variables
  ros::NodeHandle pnh_;
  ros::Subscriber path_subscriber_;
  ros::Subscriber map_subscriber_;
  ros::Publisher turn_signal_publisher_;
  ros::Timer vehicle_pose_timer_;
  ros::Timer turn_signal_timer_;

  // input data
  DataManager data_;
  TurnSignalParameters parameters_;

  // callbacks
  void onTurnSignalTimer(const ros::TimerEvent& event);

  // turn signal factors
  bool isChangingLane(const autoware_planning_msgs::PathWithLaneId& path, const FrenetCoordinate3d& vehicle_pose_frenet,
                      autoware_vehicle_msgs::TurnSignal* signal_state_ptr, double* distance_ptr) const;
  bool isTurning(const autoware_planning_msgs::PathWithLaneId& path, const FrenetCoordinate3d& vehicle_pose_frenet,
                 autoware_vehicle_msgs::TurnSignal* signal_state_ptr, double* distance_ptr) const;

  // other
  lanelet::routing::RelationType getRelation(const lanelet::ConstLanelet& prev_lane,
                                             const lanelet::ConstLanelet& next_lane) const;

public:
  TurnSignalDecider();
};
}  // namespace turn_signal_decider

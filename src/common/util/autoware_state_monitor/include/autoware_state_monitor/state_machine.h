#pragma once

#include <deque>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_system_msgs/AutowareState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>

#include "autoware_state.h"
#include "config.h"

struct StateInput {
  TopicStats topic_stats;
  ParamStats param_stats;
  TfStats tf_stats;

  geometry_msgs::PoseStamped::ConstPtr current_pose;
  geometry_msgs::Pose::ConstPtr goal_pose;

  std_msgs::Bool::ConstPtr autoware_engage;
  std_msgs::Bool::ConstPtr vehicle_engage;
  autoware_planning_msgs::Route::ConstPtr route;
  geometry_msgs::TwistStamped::ConstPtr twist;
  std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_buffer;
};

struct StateParam {
  double th_arrived_distance_m;
  double th_stopped_time_sec;
  double th_stopped_velocity_mps;
};

struct Times {
  ros::Time arrived_goal;
};

class StateMachine {
 public:
  explicit StateMachine(const StateParam& state_param) : state_param_(state_param) {}

  AutowareState getCurrentState() const { return autoware_state_; }
  AutowareState updateState(const StateInput& state_input);

 private:
  AutowareState autoware_state_ = AutowareState::InitializingVehicle;
  StateInput state_input_;
  const StateParam state_param_;

  mutable Times times_;
  mutable autoware_planning_msgs::Route::ConstPtr executing_route_;

  AutowareState judgeAutowareState() const;

  bool isModuleInitialized(const char* module_name) const;
  bool isVehicleInitialized() const;
  bool isRouteReceived() const;
  bool isNewRouteReceived() const;
  bool isPlanningCompleted() const;
  bool isEngaged() const;
  bool isOverrided() const;
  bool hasArrivedGoal() const;
  bool hasFailedToArriveGoal() const;
};

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

  std_msgs::Bool::ConstPtr autoware_engage;
  std_msgs::Bool::ConstPtr vehicle_engage;
  autoware_planning_msgs::Route::ConstPtr route;
  geometry_msgs::TwistStamped::ConstPtr twist;
  std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_buffer;
};

class StateMachine {
 public:
  AutowareState getCurrentState() const { return autoware_state_; }
  AutowareState getNextState(const StateInput& state_input);

 private:
  AutowareState autoware_state_ = AutowareState::InitializingVehicle;
  StateInput state_input_;

  AutowareState judgeAutowareState() const;

  bool isModuleInitialized(const char* module_name) const;
  bool isVehicleInitialized() const;
  bool isRouteReceived() const;
  bool isPlanningCompleted() const;
  bool isEngaged() const;
  bool isOverrided() const;
  bool hasArrivedGoal() const;
  bool hasFailedToArriveGoal() const;
};

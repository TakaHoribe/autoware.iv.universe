#pragma once

#include <autoware_system_msgs/AutowareState.h>

enum class AutowareState : int8_t {
  Error = -1,
  InitializingVehicle = 0,
  WaitingForRoute,
  Planning,
  WaitingForEngage,
  Driving,
  ArrivedGoal,
  FailedToArriveGoal,
};

inline AutowareState fromMsg(const autoware_system_msgs::AutowareState& msg) {
  if (msg.state == autoware_system_msgs::AutowareState::Error) return AutowareState::Error;
  if (msg.state == autoware_system_msgs::AutowareState::InitializingVehicle) return AutowareState::InitializingVehicle;
  if (msg.state == autoware_system_msgs::AutowareState::WaitingForRoute) return AutowareState::WaitingForRoute;
  if (msg.state == autoware_system_msgs::AutowareState::Planning) return AutowareState::Planning;
  if (msg.state == autoware_system_msgs::AutowareState::WaitingForEngage) return AutowareState::WaitingForEngage;
  if (msg.state == autoware_system_msgs::AutowareState::Driving) return AutowareState::Driving;
  if (msg.state == autoware_system_msgs::AutowareState::ArrivedGoal) return AutowareState::ArrivedGoal;
  if (msg.state == autoware_system_msgs::AutowareState::FailedToArriveGoal) return AutowareState::FailedToArriveGoal;
  return AutowareState::Error;
}

inline autoware_system_msgs::AutowareState toMsg(const AutowareState& state) {
  autoware_system_msgs::AutowareState msg;

  msg.state = [&state]() {
    if (state == AutowareState::Error) return autoware_system_msgs::AutowareState::Error;
    if (state == AutowareState::InitializingVehicle) return autoware_system_msgs::AutowareState::InitializingVehicle;
    if (state == AutowareState::WaitingForRoute) return autoware_system_msgs::AutowareState::WaitingForRoute;
    if (state == AutowareState::Planning) return autoware_system_msgs::AutowareState::Planning;
    if (state == AutowareState::WaitingForEngage) return autoware_system_msgs::AutowareState::WaitingForEngage;
    if (state == AutowareState::Driving) return autoware_system_msgs::AutowareState::Driving;
    if (state == AutowareState::ArrivedGoal) return autoware_system_msgs::AutowareState::ArrivedGoal;
    if (state == AutowareState::FailedToArriveGoal) return autoware_system_msgs::AutowareState::FailedToArriveGoal;
    return autoware_system_msgs::AutowareState::Error;
  }();

  return msg;
}

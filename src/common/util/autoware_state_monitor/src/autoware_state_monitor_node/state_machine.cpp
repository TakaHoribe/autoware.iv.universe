#include <autoware_state_monitor/state_machine.h>

namespace {

double calcDistance2d(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
  return std::hypot(p1.x - p2.x, p1.y - p2.y);
}

double calcDistance2d(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
  return calcDistance2d(p1.position, p2.position);
}

bool isNearGoal(const geometry_msgs::Pose& current_pose, const geometry_msgs::Pose& goal_pose, const double th_dist) {
  return calcDistance2d(current_pose, goal_pose) < th_dist;
}

bool isStopped(const std::deque<geometry_msgs::TwistStamped::ConstPtr>& twist_buffer,
               const double th_stopped_velocity_mps) {
  for (const auto& twist : twist_buffer) {
    if (std::abs(twist->twist.linear.x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

template <class T>
std::vector<T> filterConfigByModuleName(const std::vector<T>& configs, const char* module_name) {
  std::vector<T> filtered;

  for (const auto& config : configs) {
    if (config.module == module_name) {
      filtered.push_back(config);
    }
  }

  return filtered;
}

}  // namespace

struct ModuleName {
  static constexpr const char* Sensing = "sensing";
  static constexpr const char* Localization = "localization";
  static constexpr const char* Perception = "perception";
  static constexpr const char* Planning = "planning";
  static constexpr const char* Control = "control";
};

bool StateMachine::isModuleInitialized(const char* module_name) const {
  const auto non_received_topics = filterConfigByModuleName(state_input_.topic_stats.non_received_list, module_name);
  const auto non_set_params = filterConfigByModuleName(state_input_.param_stats.non_set_list, module_name);
  const auto non_received_tfs = filterConfigByModuleName(state_input_.tf_stats.non_received_list, module_name);

  if (non_received_topics.empty() && non_set_params.empty() && non_received_tfs.empty()) {
    return true;
  }

  for (const auto& topic_config : non_received_topics) {
    ROS_WARN("topic `%s` is not received yet", topic_config.name.c_str());
  }

  for (const auto& param_config : non_set_params) {
    ROS_WARN("param `%s` is not set", param_config.name.c_str());
  }

  for (const auto& tf_config : non_received_tfs) {
    ROS_WARN("tf from `%s` to `%s` is not received yet", tf_config.from.c_str(), tf_config.to.c_str());
  }

  ROS_INFO("module `%s` is not initialized", module_name);

  return false;
}

bool StateMachine::isVehicleInitialized() const {
  if (!isModuleInitialized(ModuleName::Sensing)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::Localization)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::Perception)) {
    return false;
  }

  return true;
}

bool StateMachine::isRouteReceived() const { return state_input_.route != nullptr; }

bool StateMachine::isPlanningCompleted() const {
  if (!isModuleInitialized(ModuleName::Planning)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::Control)) {
    return false;
  }

  return true;
}

bool StateMachine::isEngaged() const {
  // TODO: Output engage status from controller_interface instead of topic
  if (!state_input_.autoware_engage) {
    return false;
  }

  if (state_input_.autoware_engage->data != 1) {
    return false;
  }

  if (!state_input_.vehicle_engage) {
    return false;
  }

  if (state_input_.vehicle_engage->data != 1) {
    return false;
  }

  return true;
}

bool StateMachine::isOverrided() const { return !isEngaged(); }

bool StateMachine::hasArrivedGoal() const {
  const auto is_near_goal =
      isNearGoal(state_input_.current_pose->pose, *state_input_.goal_pose, state_param_.th_arrived_distance_m);
  const auto is_stopped = isStopped(state_input_.twist_buffer, state_param_.th_stopped_velocity_mps);

  if (is_near_goal && is_stopped) {
    return true;
  }

  return false;
}

bool StateMachine::hasFailedToArriveGoal() const {
  // not implemented
  return false;
}

AutowareState StateMachine::updateState(const StateInput& state_input) {
  state_input_ = state_input;
  autoware_state_ = judgeAutowareState();
  return autoware_state_;
}

AutowareState StateMachine::judgeAutowareState() const {
  switch (autoware_state_) {
    case (AutowareState::InitializingVehicle): {
      if (!isVehicleInitialized()) {
        break;
      }

      return AutowareState::WaitingForRoute;
    }

    case (AutowareState::WaitingForRoute): {
      // TODO: canExecuteAutonomousDriving, inGeoFence, etc...?

      if (!isRouteReceived()) {
        break;
      }

      if (state_input_.route == executing_route_) {
        break;
      }

      executing_route_ = state_input_.route;

      return AutowareState::Planning;
    }

    case (AutowareState::Planning): {
      if (!isPlanningCompleted()) {
        break;
      }

      return AutowareState::WaitingForEngage;
    }

    case (AutowareState::WaitingForEngage): {
      if (!isEngaged()) {
        break;
      }

      return AutowareState::Driving;
    }

    case (AutowareState::Driving): {
      if (isOverrided()) {
        return AutowareState::WaitingForEngage;
      }

      if (hasArrivedGoal()) {
        times_.arrived_goal = ros::Time::now();
        return AutowareState::ArrivedGoal;
      }

      if (hasFailedToArriveGoal()) {
        return AutowareState::FailedToArriveGoal;
      }

      break;
    }

    case (AutowareState::ArrivedGoal): {
      if (isOverrided()) {
        return AutowareState::WaitingForEngage;
      }

      constexpr double wait_time_after_arrived_goal = 2;
      const auto time_from_arrived_goal = ros::Time::now() - times_.arrived_goal;
      if (time_from_arrived_goal.toSec() > wait_time_after_arrived_goal) {
        return AutowareState::WaitingForRoute;
      }

      break;
    }

    case (AutowareState::FailedToArriveGoal): {
      if (isOverrided()) {
        return AutowareState::WaitingForEngage;
      }

      constexpr double wait_time_after_failed = 2;
      const auto time_from_failed = ros::Time::now() - times_.arrived_goal;
      if (time_from_failed.toSec() > wait_time_after_failed) {
        return AutowareState::Error;
      }

      break;
    }

    case (AutowareState::Error): {
      break;
    }

    default: {
      std::ostringstream oss;
      ROS_ERROR("no state was given: state = %d", static_cast<int>(autoware_state_));

      return AutowareState::Error;
    }
  }

  // continue previous state when break
  return autoware_state_;
}

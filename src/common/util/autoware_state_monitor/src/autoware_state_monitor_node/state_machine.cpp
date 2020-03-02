#include <autoware_state_monitor/state_machine.h>

struct ModuleName {
  static constexpr const char* Sensing = "sensing";
  static constexpr const char* Localization = "localization";
  static constexpr const char* Perception = "perception";
  static constexpr const char* Planning = "planning";
  static constexpr const char* Control = "control";
};

bool StateMachine::isModuleInitialized(const char* module_name) const {
  // Non received topic
  {
    std::vector<TopicConfig> non_received_list;
    for (const auto& topic_config : state_input_.topic_stats.non_received_list) {
      if (topic_config.module != module_name) {
        continue;
      }

      ROS_WARN("topic `%s` is not received yet", topic_config.name.c_str());
      non_received_list.push_back(topic_config);
    }

    if (!non_received_list.empty()) {
      return false;
    }
  }

  // Non set param
  {
    std::vector<ParamConfig> non_set_list;
    for (const auto& param_config : state_input_.param_stats.non_set_list) {
      if (param_config.module != module_name) {
        continue;
      }

      ROS_WARN("param `%s` is not set", param_config.name.c_str());
      non_set_list.push_back(param_config);
    }

    if (!non_set_list.empty()) {
      return false;
    }
  }

  // Non received TF
  {
    std::vector<TfConfig> non_received_list;
    for (const auto& tf_config : state_input_.tf_stats.non_received_list) {
      if (tf_config.module != module_name) {
        continue;
      }

      ROS_WARN("tf from `%s` to `%s` is not received yet", tf_config.from.c_str(), tf_config.to.c_str());
      non_received_list.push_back(tf_config);
    }

    if (!non_received_list.empty()) {
      return false;
    }
  }

  return true;
}

bool StateMachine::isVehicleInitialized() const {
  ROS_DEBUG("isVehicleInitialized");

  // Sensing
  if (!isModuleInitialized(ModuleName::Sensing)) {
    ROS_INFO("Sensing module is not initialized");
    return false;
  }

  // Localization
  if (!isModuleInitialized(ModuleName::Localization)) {
    ROS_INFO("Localization module is not initialized");
    return false;
  }

  // Perception
  if (!isModuleInitialized(ModuleName::Perception)) {
    ROS_INFO("Perception module is not initialized");
    return false;
  }

  return true;
}

bool StateMachine::isRouteReceived() const {
  ROS_DEBUG("isRouteReceived");

  return state_input_.route != nullptr;
}

bool StateMachine::isPlanningCompleted() const {
  ROS_DEBUG("isPlanningCompleted");

  if (!isModuleInitialized(ModuleName::Planning)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::Control)) {
    return false;
  }

  return true;
}

bool StateMachine::isEngaged() const {
  ROS_DEBUG("isEngaged");

  // Disengage on launch?

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

bool StateMachine::isOverrided() const {
  ROS_DEBUG("isOverrided");

  return !isEngaged();
}

bool StateMachine::hasArrivedGoal() const {
  ROS_DEBUG("hasArrivedGoal: not implemented");

  return false;
}

bool StateMachine::hasFailedToArriveGoal() const {
  ROS_DEBUG("hasArrivedGoal: not implemented");

  return false;
}

AutowareState StateMachine::getNextState(const StateInput& state_input) {
  state_input_ = state_input;
  return judgeAutowareState();
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

      // Move to WaitingForEngage after a while?

      break;
    }

    case (AutowareState::FailedToArriveGoal): {
      if (isOverrided()) {
        return AutowareState::WaitingForEngage;
      }

      // Move to Error after a while?

      break;
    }

    case (AutowareState::Error): {
      // TODO: error handling
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

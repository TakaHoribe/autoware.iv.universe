#include <autoware_state_monitor/autoware_state_monitor_node.h>

bool AutowareStateMonitorNode::isModuleInitialized(const char* module_name) const {
  // Non received topic
  {
    std::vector<TopicConfig> non_received_list;
    for (const auto& topic_config : topic_stats_.non_received_list) {
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
    for (const auto& param_config : param_stats_.non_set_list) {
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
    for (const auto& tf_config : tf_stats_.non_received_list) {
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

bool AutowareStateMonitorNode::isVehicleInitialized() const {
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

bool AutowareStateMonitorNode::isRouteReceived() const {
  ROS_DEBUG("isRouteReceived");

  return route_ != nullptr;
}

bool AutowareStateMonitorNode::isPlanningCompleted() const {
  ROS_DEBUG("isPlanningCompleted");

  if (!isModuleInitialized(ModuleName::Planning)) {
    return false;
  }

  if (!isModuleInitialized(ModuleName::Control)) {
    return false;
  }

  return true;
}

bool AutowareStateMonitorNode::isEngaged() const {
  ROS_DEBUG("isEngaged");

  // Disengage on launch?

  if (!autoware_engage_) {
    return false;
  }

  if (autoware_engage_->data != 1) {
    return false;
  }

  if (!vehicle_engage_) {
    return false;
  }

  if (vehicle_engage_->data != 1) {
    return false;
  }

  return true;
}

bool AutowareStateMonitorNode::isOverrided() const {
  ROS_DEBUG("isOverrided");

  return !isEngaged();
}

bool AutowareStateMonitorNode::hasArrivedGoal() const {
  ROS_DEBUG("hasArrivedGoal: not implemented");

  return false;
}

bool AutowareStateMonitorNode::hasFailedToArriveGoal() const {
  ROS_DEBUG("hasArrivedGoal: not implemented");

  return false;
}

AutowareState AutowareStateMonitorNode::judgeAutowareState() const {
  switch (autoware_state_) {
    case (AutowareState::InitializingVehicle): {
      if (!isVehicleInitialized()) {
        return AutowareState::InitializingVehicle;
      }

      return AutowareState::WaitingForRoute;
    }

    case (AutowareState::WaitingForRoute): {
      // TODO: canExecuteAutonomousDriving, inGeoFence, etc...?

      if (!isRouteReceived()) {
        return AutowareState::WaitingForRoute;
      }

      return AutowareState::Planning;
    }

    case (AutowareState::Planning): {
      if (!isPlanningCompleted()) {
        return AutowareState::Planning;
      }

      return AutowareState::WaitingForEngage;
    }

    case (AutowareState::WaitingForEngage): {
      if (!isEngaged()) {
        return AutowareState::WaitingForEngage;
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

      return AutowareState::Driving;
    }

    case (AutowareState::ArrivedGoal): {
      if (isOverrided()) {
        return AutowareState::WaitingForEngage;
      }

      // Move to WaitingForEngage after a while?

      return AutowareState::ArrivedGoal;
    }

    case (AutowareState::FailedToArriveGoal): {
      if (isOverrided()) {
        return AutowareState::WaitingForEngage;
      }

      // Move to Error after a while?

      return AutowareState::FailedToArriveGoal;
    }

    case (AutowareState::Error): {
      // TODO: error handling
      return AutowareState::Error;
    }

    default: {
      std::ostringstream oss;
      ROS_ERROR("no state was given: state = %d", static_cast<int>(autoware_state_));

      return AutowareState::Error;
    }
  }
}

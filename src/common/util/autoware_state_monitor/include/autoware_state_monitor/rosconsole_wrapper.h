#pragma once

#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <ros/ros.h>

#include <fmt/format.h>

inline void logThrottleNamed(const ros::console::levels::Level& log_level, const double duration,
                             const std::string& log_name, const std::string& msg) {
  static std::unordered_map<std::string, ros::Time> last_output_time;

  if (last_output_time.count(log_name) != 0) {
    const auto time_from_last_output = ros::Time::now() - last_output_time.at(log_name);
    if (time_from_last_output.toSec() < duration) {
      return;
    }
  }

  last_output_time[log_name] = ros::Time::now();
  ROS_LOG_STREAM(log_level, ROSCONSOLE_DEFAULT_NAME, msg);
}

#pragma once

#include <deque>
#include <memory>
#include <string>

#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <topic_tools/shape_shifter.h>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Trajectory.h>
#include <autoware_system_msgs/AutowareState.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Bool.h>

struct TopicConfig {
  explicit TopicConfig(XmlRpc::XmlRpcValue& value)
      : module(static_cast<std::string>(value["module"])),
        name(static_cast<std::string>(value["name"])),
        timeout(static_cast<double>(value["timeout"])),
        warn_rate(static_cast<double>(value["warn_rate"])) {}

  const std::string module;
  const std::string name;
  const double timeout;
  const double warn_rate;
};

struct ParamConfig {
  explicit ParamConfig(XmlRpc::XmlRpcValue& value)
      : module(static_cast<std::string>(value["module"])), name(static_cast<std::string>(value["name"])) {}

  const std::string module;
  const std::string name;
};

struct TfConfig {
  explicit TfConfig(XmlRpc::XmlRpcValue& value)
      : module(static_cast<std::string>(value["module"])),
        from(static_cast<std::string>(value["from"])),
        to(static_cast<std::string>(value["to"])),
        timeout(static_cast<double>(value["timeout"])) {}

  const std::string module;
  const std::string from;
  const std::string to;
  const double timeout;
};

struct TopicStats {
  ros::Time checked_time;
  std::vector<TopicConfig> non_received_list;
  std::vector<std::pair<TopicConfig, ros::Time>> timeout_list;  // pair<TfConfig, last_received>
  std::vector<std::pair<TopicConfig, double>> slow_rate_list;   // pair<TfConfig, rate>
};

struct ParamStats {
  ros::Time checked_time;
  std::vector<ParamConfig> non_set_list;
};

struct TfStats {
  ros::Time checked_time;
  std::vector<TfConfig> non_received_list;
  std::vector<std::pair<TfConfig, ros::Time>> timeout_list;  // pair<TfConfig, last_received>
};

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

struct ModuleName {
  static constexpr const char* Sensing = "sensing";
  static constexpr const char* Localization = "localization";
  static constexpr const char* Perception = "perception";
  static constexpr const char* Planning = "planning";
  static constexpr const char* Control = "control";
};

class AutowareStateMonitorNode {
 public:
  AutowareStateMonitorNode();

 private:
  // NodeHandle
  ros::NodeHandle nh_{""};
  ros::NodeHandle private_nh_{"~"};

  // Parameter
  double update_rate_;
  double th_max_message_delay_sec_;
  double th_arrived_distance_m_;
  double th_stopped_time_sec_;
  double th_stopped_velocity_mps_;
  bool disengage_on_route_;
  bool disengage_on_complete_;
  bool disengage_on_error_;

  std::vector<TopicConfig> topic_configs_;
  std::vector<ParamConfig> param_configs_;
  std::vector<TfConfig> tf_configs_;

  std::vector<std::string> regex_blacklist_topics_;
  std::vector<std::string> regex_blacklist_params_;
  std::vector<std::string> regex_blacklist_tfs_;

  // TF
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_{tf_buffer_};
  geometry_msgs::PoseStamped::ConstPtr current_pose_;

  // Subscriber
  ros::Subscriber sub_autoware_engage_;
  ros::Subscriber sub_vehicle_engage_;
  ros::Subscriber sub_route_;
  ros::Subscriber sub_twist_;

  void onAutowareEngage(const std_msgs::Bool::ConstPtr& msg);
  void onVehicleEngage(const std_msgs::Bool::ConstPtr& msg);
  void onRoute(const autoware_planning_msgs::Route::ConstPtr& msg);
  void onTwist(const geometry_msgs::TwistStamped::ConstPtr& msg);

  std_msgs::Bool::ConstPtr autoware_engage_;
  std_msgs::Bool::ConstPtr vehicle_engage_;
  autoware_planning_msgs::Route::ConstPtr route_;
  geometry_msgs::TwistStamped::ConstPtr twist_;
  std::deque<geometry_msgs::TwistStamped::ConstPtr> twist_buffer_;

  // Topic Buffer
  void onTopic(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name);
  void registerTopicCallback(const std::string& topic_name);
  void registerTopicCallbacks(const std::vector<TopicConfig>& topic_configs);

  std::map<std::string, ros::Subscriber> sub_topic_map_;
  std::map<std::string, std::deque<ros::Time>> topic_received_time_buffer_;

  // Publisher
  ros::Publisher pub_autoware_state_;

  // Timer
  void onTimer(const ros::TimerEvent& event);
  ros::Timer timer_;

  // Stats
  TopicStats getTopicStats() const;
  ParamStats getParamStats() const;
  TfStats getTfStats() const;

  TopicStats topic_stats_;
  ParamStats param_stats_;
  TfStats tf_stats_;

  // Debug
  void displayErrors() const;

  // TODO: Create StateMachine class
  // State Transition
  AutowareState autoware_state_ = AutowareState::InitializingVehicle;

  bool isModuleInitialized(const char* module_name) const;
  bool isVehicleInitialized() const;
  bool isRouteReceived() const;
  bool isPlanningCompleted() const;
  bool isEngaged() const;
  bool isOverrided() const;
  bool hasArrivedGoal() const;
  bool hasFailedToArriveGoal() const;

  AutowareState judgeAutowareState() const;
};

#include <autoware_state_monitor/autoware_state_monitor_node.h>

#include <numeric>
#include <regex>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace {

bool isBlacklistName(const std::vector<std::string>& regex_blacklist_names, const std::string& name) {
  for (const auto& regex_blacklist_name : regex_blacklist_names) {
    if (std::regex_search(name, std::regex(regex_blacklist_name))) {
      return true;
    }
  }
  return false;
}

template <class Config>
std::vector<Config> getConfigs(const ros::NodeHandle& nh, const std::string& config_name) {
  XmlRpc::XmlRpcValue xml;
  if (!nh.getParam(config_name, xml)) {
    const auto msg = std::string("no parameter found: ") + config_name;
    throw std::runtime_error(msg);
  }

  std::vector<Config> configs;
  configs.reserve(xml.size());

  for (size_t i = 0; i < xml.size(); ++i) {
    auto& value = xml[i];
    configs.emplace_back(value);
  }

  return configs;
}

double calcTopicRate(const std::deque<ros::Time>& topic_received_time_buffer) {
  assert(topic_received_time_buffer.size() >= 2);

  const auto num_segments = topic_received_time_buffer.size() - 1;

  std::vector<double> topic_durations;
  for (size_t i = 0; i < num_segments; ++i) {
    const auto& t1 = topic_received_time_buffer.at(i);
    const auto& t2 = topic_received_time_buffer.at(i + 1);

    topic_durations.push_back((t2 - t1).toSec());
  }

  const auto average_duration =
      std::accumulate(std::begin(topic_durations), std::end(topic_durations), 0.0) / num_segments;

  return 1 / average_duration;
}

geometry_msgs::PoseStamped::ConstPtr getCurrentPose(const tf2_ros::Buffer& tf_buffer) {
  geometry_msgs::TransformStamped tf_current_pose;

  try {
    tf_current_pose = tf_buffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    return nullptr;
  }

  geometry_msgs::PoseStamped::Ptr p(new geometry_msgs::PoseStamped());
  p->header = tf_current_pose.header;
  p->pose.orientation = tf_current_pose.transform.rotation;
  p->pose.position.x = tf_current_pose.transform.translation.x;
  p->pose.position.y = tf_current_pose.transform.translation.y;
  p->pose.position.z = tf_current_pose.transform.translation.z;

  return geometry_msgs::PoseStamped::ConstPtr(p);
}

bool isNearGoal(const geometry_msgs::Pose& goal_pose, const geometry_msgs::Pose& current_pose, const double th_dist) {
  const auto& p1 = current_pose.position;
  const auto& p2 = goal_pose.position;

  const auto dist = std::hypot(p1.x - p2.x, p1.y - p2.y);

  return dist < th_dist;
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

}  // namespace

void AutowareStateMonitorNode::onAutowareEngage(const std_msgs::Bool::ConstPtr& msg) { autoware_engage_ = msg; }

void AutowareStateMonitorNode::onVehicleEngage(const std_msgs::Bool::ConstPtr& msg) { vehicle_engage_ = msg; }

void AutowareStateMonitorNode::onRoute(const autoware_planning_msgs::Route::ConstPtr& msg) {
  route_ = msg;

  if (disengage_on_route_) {
    // TODO: set disengage
  }
}

void AutowareStateMonitorNode::onTwist(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  twist_ = msg;

  twist_buffer_.push_back(msg);

  // Delete old data in buffer
  while (true) {
    const auto time_diff = msg->header.stamp - twist_buffer_.front()->header.stamp;

    if (time_diff.toSec() < th_stopped_time_sec_) {
      break;
    }

    twist_buffer_.pop_front();
  }
}

void AutowareStateMonitorNode::onTimer(const ros::TimerEvent& event) {
  topic_stats_ = getTopicStats();
  param_stats_ = getParamStats();
  tf_stats_ = getTfStats();

  const auto prev_autoware_state = autoware_state_;
  autoware_state_ = judgeAutowareState();

  if (autoware_state_ != prev_autoware_state) {
    ROS_INFO("state changed: %s -> %s", toMsg(prev_autoware_state).state.c_str(), toMsg(autoware_state_).state.c_str());
  }

  displayErrors();

  pub_autoware_state_.publish(toMsg(autoware_state_));
}

void AutowareStateMonitorNode::onTopic(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name) {
  const auto now = ros::Time::now();

  auto& buf = topic_received_time_buffer_.at(topic_name);
  buf.push_back(now);

  constexpr size_t topic_received_time_buffer_size = 10;
  if (buf.size() > topic_received_time_buffer_size) {
    buf.pop_front();
  }
}

void AutowareStateMonitorNode::registerTopicCallback(const std::string& topic_name) {
  // Initialize buffer
  topic_received_time_buffer_[topic_name] = {};

  // Register callback
  using Callback = boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)>;
  const auto callback = static_cast<Callback>(boost::bind(&AutowareStateMonitorNode::onTopic, this, _1, topic_name));
  sub_topic_map_[topic_name] = nh_.subscribe(topic_name, 10, callback);
}

void AutowareStateMonitorNode::registerTopicCallbacks(const std::vector<TopicConfig>& topic_configs) {
  for (const auto& topic_config : topic_configs) {
    const auto& topic_name = topic_config.name;

    if (isBlacklistName(regex_blacklist_topics_, topic_name)) {
      ROS_INFO("topic `%s` is ignored", topic_name.c_str());
      continue;
    }

    registerTopicCallback(topic_name);
  }
}

TopicStats AutowareStateMonitorNode::getTopicStats() const {
  TopicStats topic_stats;
  topic_stats.checked_time = ros::Time::now();

  for (const auto& topic_config : topic_configs_) {
    if (isBlacklistName(regex_blacklist_topics_, topic_config.name)) {
      continue;
    }

    // Alias
    const auto& buf = topic_received_time_buffer_.at(topic_config.name);

    // Check at least once received
    if (buf.empty()) {
      topic_stats.non_received_list.push_back(topic_config);
      continue;
    }

    // Check timeout
    const auto last_received_time = buf.back();
    const auto time_diff = (topic_stats.checked_time - last_received_time).toSec();
    const auto is_timeout = (topic_config.timeout != 0) && (time_diff > topic_config.timeout);
    if (is_timeout) {
      topic_stats.timeout_list.emplace_back(topic_config, last_received_time);
    }

    // Check topic rate
    if (!is_timeout && buf.size() >= 2) {
      const auto topic_rate = calcTopicRate(buf);
      if (topic_config.warn_rate != 0 && topic_rate < topic_config.warn_rate) {
        topic_stats.slow_rate_list.emplace_back(topic_config, topic_rate);
      }
    }
  }

  return topic_stats;
}

ParamStats AutowareStateMonitorNode::getParamStats() const {
  ParamStats param_stats;
  param_stats.checked_time = ros::Time::now();

  for (const auto& param_config : param_configs_) {
    if (isBlacklistName(regex_blacklist_params_, param_config.name)) {
      continue;
    }

    XmlRpc::XmlRpcValue xml;
    const auto result = nh_.getParam(param_config.name, xml);
    if (!result) {
      param_stats.non_set_list.push_back(param_config);
    }
  }

  return param_stats;
}

TfStats AutowareStateMonitorNode::getTfStats() const {
  TfStats tf_stats;
  tf_stats.checked_time = ros::Time::now();

  for (const auto& tf_config : tf_configs_) {
    if (isBlacklistName(regex_blacklist_tfs_, tf_config.from) || isBlacklistName(regex_blacklist_tfs_, tf_config.to)) {
      continue;
    }

    try {
      const auto transform = tf_buffer_.lookupTransform(tf_config.from, tf_config.to, ros::Time(0), ros::Duration(0));

      const auto last_received_time = transform.header.stamp;
      const auto time_diff = (tf_stats.checked_time - last_received_time).toSec();
      if (time_diff > tf_config.timeout) {
        tf_stats.timeout_list.emplace_back(tf_config, last_received_time);
      }
    } catch (tf2::TransformException ex) {
      tf_stats.non_received_list.push_back(tf_config);
    }
  }

  return tf_stats;
}

void AutowareStateMonitorNode::displayErrors() const {
  for (const auto& topic_config_pair : topic_stats_.timeout_list) {
    const auto& topic_config = topic_config_pair.first;
    const auto& last_received_time = topic_config_pair.second;

    ROS_ERROR("topic `%s` is timeout: timeout = %f, checked_time = %f, last_received_time = %f",
              topic_config.name.c_str(), topic_config.timeout, topic_stats_.checked_time.toSec(),
              last_received_time.toSec());
  }

  for (const auto& topic_config_pair : topic_stats_.slow_rate_list) {
    const auto& topic_config = topic_config_pair.first;
    const auto& topic_rate = topic_config_pair.second;

    ROS_WARN("topic `%s` is slow rate: warn_rate = %f, acctual_rate = %f", topic_config.name.c_str(),
             topic_config.warn_rate, topic_rate);
  }

  for (const auto& tf_config_pair : tf_stats_.timeout_list) {
    const auto& tf_config = tf_config_pair.first;
    const auto& last_received_time = tf_config_pair.second;

    ROS_ERROR("tf from `%s` to `%s` is timeout: timeout = %f, checked_time = %f, last_received_time = %f",
              tf_config.from.c_str(), tf_config.to.c_str(), tf_config.timeout, tf_stats_.checked_time.toSec(),
              last_received_time.toSec());
  }
}

AutowareStateMonitorNode::AutowareStateMonitorNode() {
  // Parameter
  private_nh_.param("update_rate", update_rate_, 10.0);
  private_nh_.param("th_max_message_delay_sec", th_max_message_delay_sec_, 0.5);
  private_nh_.param("th_arrived_distance_m", th_arrived_distance_m_, 1.0);
  private_nh_.param("th_stopped_time_sec", th_stopped_time_sec_, 1.0);
  private_nh_.param("th_stopped_velocity_mps", th_stopped_velocity_mps_, 0.01);
  private_nh_.param("disengage_on_route", disengage_on_route_, false);
  private_nh_.param("disengage_on_complete", disengage_on_complete_, false);
  private_nh_.param("disengage_on_error", disengage_on_error_, false);

  // Config
  topic_configs_ = getConfigs<TopicConfig>(private_nh_, "topic_configs");
  param_configs_ = getConfigs<ParamConfig>(private_nh_, "param_configs");
  tf_configs_ = getConfigs<TfConfig>(private_nh_, "tf_configs");

  // Blacklist
  private_nh_.param("regex_blacklist_topics", regex_blacklist_topics_, {});
  private_nh_.param("regex_blacklist_params", regex_blacklist_params_, {});
  private_nh_.param("regex_blacklist_tfs", regex_blacklist_tfs_, {});

  // Topic Callback
  registerTopicCallbacks(topic_configs_);

  // Subscriber
  sub_autoware_engage_ =
      private_nh_.subscribe("input/autoware_engage", 1, &AutowareStateMonitorNode::onAutowareEngage, this);
  sub_vehicle_engage_ =
      private_nh_.subscribe("input/vehicle_engage", 1, &AutowareStateMonitorNode::onVehicleEngage, this);
  sub_route_ = private_nh_.subscribe("input/route", 1, &AutowareStateMonitorNode::onRoute, this);
  sub_twist_ = private_nh_.subscribe("input/twist", 100, &AutowareStateMonitorNode::onTwist, this);

  // Publisher
  pub_autoware_state_ = private_nh_.advertise<autoware_system_msgs::AutowareState>("output/autoware_state", 1);

  // Wait for first topics
  ros::Duration(1.0).sleep();

  // Timer
  timer_ = private_nh_.createTimer(ros::Rate(update_rate_), &AutowareStateMonitorNode::onTimer, this);
}

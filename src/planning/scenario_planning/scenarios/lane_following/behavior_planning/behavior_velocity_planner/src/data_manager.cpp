#include <lanelet2_extension/utility/message_conversion.h>
#include <behavior_velocity_planner/data_manager.hpp>

namespace behavior_planning {
SingletonDataManager::SingletonDataManager(): tf_listener_(tf_buffer_) {}

void SingletonDataManager::perceptionCallback(
    const autoware_perception_msgs::DynamicObjectArray& input_perception_msg) {
  perception_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(input_perception_msg);
}

void SingletonDataManager::pointcloudCallback(const sensor_msgs::PointCloud2& input_pointcloud_msg) {
  pointcloud_ptr_ = std::make_shared<sensor_msgs::PointCloud2>(input_pointcloud_msg);
}

void SingletonDataManager::velocityCallback(const geometry_msgs::TwistStamped& input_twist_msg) {
  vehicle_velocity_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(input_twist_msg);
}

void SingletonDataManager::mapCallback(const autoware_lanelet2_msgs::MapBin& input_map_msg) {
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(input_map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

void SingletonDataManager::trafficLightStatesCallback(
    const autoware_traffic_light_msgs::TrafficLightStateArray& input_msg) {
  for (size_t i = 0; i < input_msg.states.size(); ++i) {
    traffic_light_id_map_[input_msg.states.at(i).id] = std::make_tuple(input_msg.header, input_msg.states.at(i));
  }
}

void SingletonDataManager::setWheelBase(const double& wheel_base) {
  wheel_base_ptr_ = std::make_shared<double>(wheel_base);
}

void SingletonDataManager::setFrontOverhang(const double& front_overhang) {
  front_overhang_ptr_ = std::make_shared<double>(front_overhang);
}

void SingletonDataManager::setVehicleWidth(const double& width) {
  vehicle_width_ptr_ = std::make_shared<double>(width);
}

bool SingletonDataManager::getDynemicObjects(
    std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const>& objects) {
  if (perception_ptr_ == nullptr) return false;
  objects = perception_ptr_;
  return true;
}

bool SingletonDataManager::getNoGroundPointcloud(std::shared_ptr<sensor_msgs::PointCloud2 const>& pointcloud) {
  if (pointcloud_ptr_ == nullptr) return false;
  pointcloud = pointcloud_ptr_;
  return true;
}

bool SingletonDataManager::getTrafficLightState(const int id, std_msgs::Header& header,
                                                autoware_traffic_light_msgs::TrafficLightState& traffic_light) {
  if (traffic_light_id_map_.count(id) == 0) return false;
  header = std::get<0>(traffic_light_id_map_.at(id));
  traffic_light = std::get<1>(traffic_light_id_map_.at(id));
  return true;
}

bool SingletonDataManager::getCurrentSelfVelocity(std::shared_ptr<geometry_msgs::TwistStamped const>& twist) {
  if (vehicle_velocity_ptr_ == nullptr) return false;
  twist = vehicle_velocity_ptr_;
  return true;
}

bool SingletonDataManager::getCurrentSelfPose(geometry_msgs::PoseStamped& pose) {
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(0);
  try {
    geometry_msgs::TransformStamped transform;
    transform = tf_buffer_.lookupTransform(header.frame_id, "base_link", header.stamp, ros::Duration(0.1));
    pose.pose.position.x = transform.transform.translation.x;
    pose.pose.position.y = transform.transform.translation.y;
    pose.pose.position.z = transform.transform.translation.z;
    pose.pose.orientation.x = transform.transform.rotation.x;
    pose.pose.orientation.y = transform.transform.rotation.y;
    pose.pose.orientation.z = transform.transform.rotation.z;
    pose.pose.orientation.w = transform.transform.rotation.w;
  } catch (tf2::TransformException& ex) {
    return false;
  }
  pose.header = header;
  return true;
}

bool SingletonDataManager::getLaneletMap(lanelet::LaneletMapConstPtr& lanelet_map_ptr,
                                         lanelet::routing::RoutingGraphConstPtr& routing_graph_ptr) {
  if (lanelet_map_ptr_ == nullptr || routing_graph_ptr_ == nullptr) return false;
  lanelet_map_ptr = lanelet_map_ptr_;
  routing_graph_ptr = routing_graph_ptr_;
  return true;
}

bool SingletonDataManager::getWheelBase(double& wheel_base) {
  if (wheel_base_ptr_ == nullptr) return false;
  wheel_base = *wheel_base_ptr_;
  return true;
}

bool SingletonDataManager::getFrontOverhang(double& front_overhang) {
  if (front_overhang_ptr_ == nullptr) return false;
  front_overhang = *front_overhang_ptr_;
  return true;
}

bool SingletonDataManager::getVehicleWidth(double& width) {
  if (vehicle_width_ptr_ == nullptr) return false;
  width = *vehicle_width_ptr_;
  return true;
}

bool SingletonDataManager::getTransform(const std::string& target_frame, const std::string& source_frame, const ros::Time& time,
                  const ros::Duration timeout, geometry_msgs::TransformStamped& transform_stamped) {
  try {
    transform_stamped = tf_buffer_.lookupTransform(target_frame, source_frame, time, timeout);
  } catch (tf2::TransformException& ex) {
    return false;
  }
  return true;
}
}  // namespace behavior_planning
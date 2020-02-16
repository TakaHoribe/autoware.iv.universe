#include <behavior_velocity_planner/data_manager.hpp>

#include <lanelet2_extension/utility/message_conversion.h>

namespace behavior_planning {

void SingletonDataManager::perceptionCallback(const autoware_perception_msgs::DynamicObjectArray::ConstPtr& msg) {
  planner_data_.dynamic_objects = msg;
}

void SingletonDataManager::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
  planner_data_.no_ground_pointcloud_msg = msg;

  pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_pointcloud;
  pcl::fromROSMsg(*msg, *no_ground_pointcloud);
  planner_data_.no_ground_pointcloud = no_ground_pointcloud;
}

void SingletonDataManager::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
  planner_data_.current_velocity = msg;
}

void SingletonDataManager::mapCallback(const autoware_lanelet2_msgs::MapBin::ConstPtr& msg) {
  planner_data_.lanelet_map = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(*msg, planner_data_.lanelet_map, &planner_data_.traffic_rules,
                                         &planner_data_.routing_graph);
}

void SingletonDataManager::trafficLightStatesCallback(
    const autoware_traffic_light_msgs::TrafficLightStateArray::ConstPtr& msg) {
  for (const auto& state : msg->states) {
    traffic_light_id_map_[state.id] = std::make_tuple(msg->header, state);
  }
}

void SingletonDataManager::setWheelBase(const double& wheel_base) {
  planner_data_.wheel_base = std::make_shared<double>(wheel_base);
}

void SingletonDataManager::setFrontOverhang(const double& front_overhang) {
  planner_data_.front_overhang = std::make_shared<double>(front_overhang);
}

void SingletonDataManager::setVehicleWidth(const double& vehicle_width) {
  planner_data_.vehicle_width = std::make_shared<double>(vehicle_width);
}

void SingletonDataManager::setBaseLink2FrontLength() {
  double base_link2front;
  const bool ret = getBaselink2FrontLength(base_link2front);
  if (ret) planner_data_.base_link2front = std::make_shared<double>(base_link2front);
}

bool SingletonDataManager::getDynemicObjects(autoware_perception_msgs::DynamicObjectArray::ConstPtr& objects) {
  if (planner_data_.dynamic_objects == nullptr) return false;
  objects = planner_data_.dynamic_objects;
  return true;
}

bool SingletonDataManager::getNoGroundPointcloud(sensor_msgs::PointCloud2::ConstPtr& pointcloud) {
  if (planner_data_.no_ground_pointcloud == nullptr) return false;
  pointcloud = planner_data_.no_ground_pointcloud_msg;
  return true;
}

bool SingletonDataManager::getTrafficLightState(const int id, std_msgs::Header& header,
                                                autoware_traffic_light_msgs::TrafficLightState& traffic_light) {
  if (traffic_light_id_map_.count(id) == 0) return false;
  header = std::get<0>(traffic_light_id_map_.at(id));
  traffic_light = std::get<1>(traffic_light_id_map_.at(id));
  return true;
}

bool SingletonDataManager::getCurrentSelfVelocity(geometry_msgs::TwistStamped::ConstPtr& twist) {
  if (planner_data_.current_velocity == nullptr) return false;
  twist = planner_data_.current_velocity;
  return true;
}

bool SingletonDataManager::getCurrentSelfPose(geometry_msgs::PoseStamped& pose) {
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(0);
  pose.header = header;
  return getSelfPose(pose.pose, header);
}

bool SingletonDataManager::getLaneletMap(lanelet::LaneletMapConstPtr& lanelet_map,
                                         lanelet::routing::RoutingGraphConstPtr& routing_graph) {
  if (planner_data_.lanelet_map == nullptr || planner_data_.routing_graph == nullptr) return false;
  lanelet_map = planner_data_.lanelet_map;
  routing_graph = planner_data_.routing_graph;
  return true;
}

bool SingletonDataManager::getWheelBase(double& wheel_base) {
  if (planner_data_.wheel_base == nullptr) return false;
  wheel_base = *planner_data_.wheel_base;
  return true;
}

bool SingletonDataManager::getFrontOverhang(double& front_overhang) {
  if (planner_data_.front_overhang == nullptr) return false;
  front_overhang = *planner_data_.front_overhang;
  return true;
}

bool SingletonDataManager::getVehicleWidth(double& vehicle_width) {
  if (planner_data_.vehicle_width == nullptr) return false;
  vehicle_width = *planner_data_.vehicle_width;
  return true;
}

bool SingletonDataManager::getSelfPose(geometry_msgs::Pose& self_pose, const std_msgs::Header& header) {
  try {
    const geometry_msgs::TransformStamped transform =
        tf_buffer_.lookupTransform(header.frame_id, "base_link", header.stamp, ros::Duration(0.1));
    self_pose.position.x = transform.transform.translation.x;
    self_pose.position.y = transform.transform.translation.y;
    self_pose.position.z = transform.transform.translation.z;
    self_pose.orientation = transform.transform.rotation;
    return true;
  } catch (tf2::TransformException& ex) {
    return false;
  }
}

bool SingletonDataManager::getBaselink2FrontLength(double& length) {
  double front_overhang;
  double wheel_base;
  if (!getWheelBase(wheel_base) || !getFrontOverhang(front_overhang)) return false;
  length = front_overhang + wheel_base;
  return true;
}

bool SingletonDataManager::isVehicleStopping() {
  geometry_msgs::TwistStamped::ConstPtr twist_ptr;
  getCurrentSelfVelocity(twist_ptr);
  if (twist_ptr == nullptr) return false;
  return twist_ptr->twist.linear.x < 0.1 ? true : false;
}

}  // namespace behavior_planning

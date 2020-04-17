/*
 * Copyright 2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <lane_change_planner/data_manager.h>
#include <lanelet2_extension/utility/message_conversion.h>

namespace lane_change_planner {
DataManager::DataManager() : is_parameter_set_(false), lane_change_approval_(false), force_lane_change_(false) {
  self_pose_listener_ptr_ = std::make_shared<SelfPoseLinstener>();
}

void DataManager::perceptionCallback(const autoware_perception_msgs::DynamicObjectArray::ConstPtr& input_perception_msg_ptr) {
  perception_ptr_ = input_perception_msg_ptr;
}

void DataManager::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& input_pointcloud_msg_ptr) {
  pointcloud_ptr_ = input_pointcloud_msg_ptr;
}

void DataManager::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& input_twist_msg_ptr) {
  vehicle_velocity_ptr_ = input_twist_msg_ptr;
}

void DataManager::mapCallback(const autoware_lanelet2_msgs::MapBin& input_map_msg) {
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(input_map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

void DataManager::laneChangeApprovalCallback(const std_msgs::Bool& input_approval_msg) {
  lane_change_approval_.data = input_approval_msg.data;
  lane_change_approval_.stamp = ros::Time::now();
}

void DataManager::forceLaneChangeSignalCallback(const std_msgs::Bool& input_force_lane_change_msg) {
  force_lane_change_.data = input_force_lane_change_msg.data;
  force_lane_change_.stamp = ros::Time::now();
}

void DataManager::setLaneChangerParameters(const LaneChangerParameters& parameters) {
  is_parameter_set_ = true;
  parameters_ = parameters;
}

autoware_perception_msgs::DynamicObjectArray::ConstPtr DataManager::getDynamicObjects() {
  return perception_ptr_;
}

sensor_msgs::PointCloud2::ConstPtr DataManager::getNoGroundPointcloud() { return pointcloud_ptr_; }

geometry_msgs::TwistStamped::ConstPtr DataManager::getCurrentSelfVelocity() {
  return vehicle_velocity_ptr_;
}

geometry_msgs::PoseStamped DataManager::getCurrentSelfPose() {
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(0);
  self_pose_listener_ptr_->getSelfPose(self_pose_.pose, header);
  self_pose_.header = header;

  return self_pose_;
}

LaneChangerParameters DataManager::getLaneChangerParameters() { return parameters_; }

bool DataManager::getLaneChangeApproval() {
  constexpr double timeout = 0.5;
  if (ros::Time::now() - lane_change_approval_.stamp > ros::Duration(timeout)) {
    return false;
  } else {
    return lane_change_approval_.data;
  }
}

bool DataManager::getForceLaneChangeSignal() {
  constexpr double timeout = 0.5;
  if (ros::Time::now() - force_lane_change_.stamp > ros::Duration(timeout)) {
    return false;
  } else {
    return force_lane_change_.data;
  }
}

bool DataManager::isDataReady() {
  if (!perception_ptr_) {
    return false;
  }
  if (!vehicle_velocity_ptr_) {
    return false;
  }
  if (!self_pose_listener_ptr_->isSelfPoseReady()) {
    return false;
  }
  return true;
}

SelfPoseLinstener::SelfPoseLinstener() : tf_listener_(tf_buffer_){};

bool SelfPoseLinstener::isSelfPoseReady() {
  return tf_buffer_.canTransform("map", "base_link", ros::Time(0), ros::Duration(0.1));
}

bool SelfPoseLinstener::getSelfPose(geometry_msgs::Pose& self_pose, const std_msgs::Header& header) {
  try {
    geometry_msgs::TransformStamped transform;
    transform = tf_buffer_.lookupTransform(header.frame_id, "base_link", header.stamp, ros::Duration(0.1));
    self_pose.position.x = transform.transform.translation.x;
    self_pose.position.y = transform.transform.translation.y;
    self_pose.position.z = transform.transform.translation.z;
    self_pose.orientation.x = transform.transform.rotation.x;
    self_pose.orientation.y = transform.transform.rotation.y;
    self_pose.orientation.z = transform.transform.rotation.z;
    self_pose.orientation.w = transform.transform.rotation.w;
    return true;
  } catch (tf2::TransformException& ex) {
    ROS_ERROR_STREAM_THROTTLE(1, "failed to find self pose :" << ex.what());
    return false;
  }
}
}  // namespace lane_change_planner
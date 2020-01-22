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

namespace lane_change_planner
{
SingletonDataManager::SingletonDataManager() : is_parameter_set_(false), lane_change_approval_(false)
{
  self_pose_listener_ptr_ = std::make_shared<SelfPoseLinstener>();
}

void SingletonDataManager::perceptionCallback(const autoware_perception_msgs::DynamicObjectArray& input_perception_msg)
{
  perception_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(input_perception_msg);
}

void SingletonDataManager::pointcloudCallback(const sensor_msgs::PointCloud2& input_pointcloud_msg)
{
  pointcloud_ptr_ = std::make_shared<sensor_msgs::PointCloud2>(input_pointcloud_msg);
}

void SingletonDataManager::velocityCallback(const geometry_msgs::TwistStamped& input_twist_msg)
{
  vehicle_velocity_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(input_twist_msg);
}

void SingletonDataManager::mapCallback(const autoware_lanelet2_msgs::MapBin& input_map_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(input_map_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
}

void SingletonDataManager::laneChangeApprovalCallback(const std_msgs::Bool& input_approval_msg)
{
  lane_change_approval_ = input_approval_msg.data;
}

void SingletonDataManager::setLaneChangerParameters(const LaneChangerParameters& parameters)
{
  is_parameter_set_ = true;
  parameters_ = parameters;
}


bool SingletonDataManager::getDynamicObjects(
    std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const>& objects)
{
  if (perception_ptr_ == nullptr)
    return false;
  objects = perception_ptr_;
  return true;
}

bool SingletonDataManager::getNoGroundPointcloud(std::shared_ptr<sensor_msgs::PointCloud2 const>& pointcloud)
{
  if (pointcloud_ptr_ == nullptr)
    return false;
  pointcloud = pointcloud_ptr_;
  return true;
}

bool SingletonDataManager::getCurrentSelfVelocity(std::shared_ptr<geometry_msgs::TwistStamped const>& twist)
{
  if (vehicle_velocity_ptr_ == nullptr)
    return false;
  twist = vehicle_velocity_ptr_;
  return true;
}

bool SingletonDataManager::getCurrentSelfPose(geometry_msgs::PoseStamped& pose)
{
  if (self_pose_listener_ptr_ == nullptr)
    return false;
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time(0);
  pose.header = header;
  return self_pose_listener_ptr_->getSelfPose(pose.pose, header);
}

bool SingletonDataManager::getLaneletMap(lanelet::LaneletMapConstPtr& lanelet_map_ptr,
                                         lanelet::routing::RoutingGraphConstPtr& routing_graph_ptr)
{
  if (lanelet_map_ptr_ == nullptr || routing_graph_ptr_ == nullptr)
    return false;
  lanelet_map_ptr = lanelet_map_ptr_;
  routing_graph_ptr = routing_graph_ptr_;
  return true;
}

SelfPoseLinstener::SelfPoseLinstener() : tf_listener_(tf_buffer_){};

bool SelfPoseLinstener::getSelfPose(geometry_msgs::Pose& self_pose, const std_msgs::Header& header)
{
  try
  {
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
  }
  catch (tf2::TransformException& ex)
  {
    return false;
  }
}

bool SingletonDataManager::getLaneChangerParameters(LaneChangerParameters& parameters)
{
  if (!is_parameter_set_)
  {
    return false;
  }
  parameters = parameters_;
  return true;
}

bool SingletonDataManager::getLaneChangeApproval()
{
  return lane_change_approval_;
}

}  // namespace lane_change_planner
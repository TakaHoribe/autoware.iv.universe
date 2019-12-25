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
 *
 * Authors: Yukihiro Saito
 *
 */

#include <ros/ros.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_extension/visualization/visualization.h>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <autoware_traffic_light_msgs/TrafficLightRoi.h>
#include <traffic_light_map_based_detector/node.hpp>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace traffic_light
{

MapBasedDetector::MapBasedDetector() : nh_(""), pnh_("~"), tf_listener_(tf_buffer_)
{
  map_sub_ = pnh_.subscribe("input/lanelet_map_bin", 1, &MapBasedDetector::mapCallback, this);
  camera_info_sub_ = pnh_.subscribe("input/camera_info", 1, &MapBasedDetector::cameraInfoCallback, this);
  route_sub_ = pnh_.subscribe("input/route", 1, &MapBasedDetector::routeCallback, this);
  roi_pub_ = pnh_.advertise<autoware_traffic_light_msgs::TrafficLightRoiArray>("output/rois", 1);
  pnh_.getParam("max_vibration_pitch", config_.max_vibration_pitch);
  pnh_.getParam("max_vibration_yaw", config_.max_vibration_yaw);
  pnh_.getParam("max_vibration_height", config_.max_vibration_height);
  pnh_.getParam("max_vibration_width", config_.max_vibration_width);
  pnh_.getParam("max_vibration_depth", config_.max_vibration_depth);
}

void MapBasedDetector::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr &input_msg)
{
  camera_info_ptr_ = input_msg;
  if (lanelet_map_ptr_ == nullptr || camera_info_ptr_ == nullptr)
    return;

  autoware_traffic_light_msgs::TrafficLightRoiArray output_msg;
  output_msg.header = camera_info_ptr_->header;
  geometry_msgs::Pose camera_pose;
  try
  {
    geometry_msgs::TransformStamped transform;
    transform = tf_buffer_.lookupTransform("map", input_msg->header.frame_id,
                                           input_msg->header.stamp, ros::Duration(0.2));
    camera_pose.position.x = transform.transform.translation.x;
    camera_pose.position.y = transform.transform.translation.y;
    camera_pose.position.z = transform.transform.translation.z;
    camera_pose.orientation.x = transform.transform.rotation.x;
    camera_pose.orientation.y = transform.transform.rotation.y;
    camera_pose.orientation.z = transform.transform.rotation.z;
    camera_pose.orientation.w = transform.transform.rotation.w;
  }
  catch (tf2::TransformException &ex)
  {
    ROS_WARN("cannot get transform frome map frame to camera frame");
    return;
  }

  const sensor_msgs::CameraInfo &camera_info = *input_msg;
  std::vector<lanelet::AutowareTrafficLightConstPtr> visible_traffic_light;
  if (all_traffic_light_ptr_ == nullptr)
    return;
  isInVisibility(*all_traffic_light_ptr_, camera_pose, camera_info, visible_traffic_light);

  for (auto tl_itr = visible_traffic_light.begin(); tl_itr != visible_traffic_light.end(); ++tl_itr)
  {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (auto lsp : lights)
    {
      if (!lsp.isLineString()) // traffic ligths must be linestrings
        continue;

      lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);
      autoware_traffic_light_msgs::TrafficLightRoi tl_roi;
      if (!getTrafficLightRoi(camera_pose, camera_info, ls, config_, tl_roi))
        continue;

      output_msg.rois.push_back(tl_roi);
    }
  }
  roi_pub_.publish(output_msg);
}

bool MapBasedDetector::getTrafficLightRoi(const geometry_msgs::Pose &camera_pose,
                                          const sensor_msgs::CameraInfo &camera_info,
                                          const lanelet::ConstLineString3d traffic_light,
                                          const Config &config,
                                          autoware_traffic_light_msgs::TrafficLightRoi &tl_roi)
{

  const double tl_height = traffic_light.attributeOr("height", 0.0);
  const double &fx = camera_info.K[(0 * 3) + 0];
  const double &fy = camera_info.K[(1 * 3) + 1];
  const double &cx = camera_info.K[(0 * 3) + 2];
  const double &cy = camera_info.K[(1 * 3) + 2];
  const auto &tl_left_down_point = traffic_light.front();
  const auto &tl_right_down_point = traffic_light.back();
  tf2::Transform tf_map2camera(tf2::Quaternion(camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z, camera_pose.orientation.w),
                               tf2::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
  // for roi.x_offset and roi.y_offset
  {
    tf2::Transform tf_map2tl(tf2::Quaternion(0, 0, 0, 1),
                             tf2::Vector3(tl_left_down_point.x(), tl_left_down_point.y(), tl_left_down_point.z() + tl_height));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

    const double &camera_x = tf_camera2tl.getOrigin().x() -
                             (std::sin(config.max_vibration_yaw / 2.0) * tf_camera2tl.getOrigin().z()) -
                             config.max_vibration_width / 2.0;
    const double &camera_y = tf_camera2tl.getOrigin().y() -
                             (std::sin(config.max_vibration_pitch / 2.0) * tf_camera2tl.getOrigin().z()) -
                             config.max_vibration_height / 2.0;
    const double &camera_z = tf_camera2tl.getOrigin().z() - config.max_vibration_depth / 2.0;
    if (camera_z <= 0.0)
      return false;
    const double image_u = (fx * camera_x + cx * camera_z) / camera_z;
    const double image_v = (fy * camera_y + cy * camera_z) / camera_z;

    tl_roi.roi.x_offset = std::max(std::min(image_u, (double)camera_info.width), 0.0);
    tl_roi.roi.y_offset = std::max(std::min(image_v, (double)camera_info.height), 0.0);
  }

  // for roi.width and roi.height
  {
    tf2::Transform tf_map2tl(tf2::Quaternion(0, 0, 0, 1),
                             tf2::Vector3(tl_right_down_point.x(), tl_right_down_point.y(), tl_right_down_point.z()));
    tf2::Transform tf_camera2tl;
    tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

    const double &camera_x = tf_camera2tl.getOrigin().x() +
                             (std::sin(config.max_vibration_yaw / 2.0) * tf_camera2tl.getOrigin().z()) +
                             config.max_vibration_width / 2.0;
    const double &camera_y = tf_camera2tl.getOrigin().y() +
                             (std::sin(config.max_vibration_pitch / 2.0) * tf_camera2tl.getOrigin().z()) +
                             config.max_vibration_height / 2.0;
    const double &camera_z = tf_camera2tl.getOrigin().z() - config.max_vibration_depth / 2.0;
    if (camera_z <= 0.0)
      return false;
    const double image_u = (fx * camera_x + cx * camera_z) / camera_z;
    const double image_v = (fy * camera_y + cy * camera_z) / camera_z;
    tl_roi.roi.width = std::max(std::min(image_u, (double)camera_info.width), 0.0) - tl_roi.roi.x_offset;
    tl_roi.roi.height = std::max(std::min(image_v, (double)camera_info.height), 0.0) - tl_roi.roi.y_offset;
    if (tl_roi.roi.width <= 0 || tl_roi.roi.height <= 0)
      return false;
  }
  return true;
}

void MapBasedDetector::mapCallback(const autoware_lanelet2_msgs::MapBin &input_msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(input_msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(lanelet_map_ptr_);
  all_traffic_light_ptr_ = std::make_shared<std::vector<lanelet::AutowareTrafficLightConstPtr>>(lanelet::utils::query::autowareTrafficLights(all_lanelets));
}

void MapBasedDetector::routeCallback(const autoware_planning_msgs::Route::ConstPtr &input_msg)
{
}

void MapBasedDetector::isInVisibility(
    const std::vector<lanelet::AutowareTrafficLightConstPtr> &all_traffic_light,
    const geometry_msgs::Pose &camera_pose,
    const sensor_msgs::CameraInfo &camera_info,
    std::vector<lanelet::AutowareTrafficLightConstPtr> &visible_traffic_light)
{
  // for each traffic light in map check if in range and in view angle of camera
  for (auto tl_itr = all_traffic_light.begin(); tl_itr != all_traffic_light.end(); ++tl_itr)
  {
    lanelet::AutowareTrafficLightConstPtr tl = *tl_itr;

    auto lights = tl->trafficLights();
    for (auto lsp : lights)
    {
      if (!lsp.isLineString()) // traffic ligths must be linestrings
        continue;

      lanelet::ConstLineString3d ls = static_cast<lanelet::ConstLineString3d>(lsp);

      const auto &tl_left_down_point = ls.front();
      const auto &tl_right_down_point = ls.back();
      const double tl_height = ls.attributeOr("height", 0.0);

      // check distance range
      geometry_msgs::Point tl_central_point;
      tl_central_point.x = (tl_right_down_point.x() + tl_left_down_point.x()) / 2.0;
      tl_central_point.y = (tl_right_down_point.y() + tl_left_down_point.y()) / 2.0;
      tl_central_point.z = (tl_right_down_point.z() + tl_left_down_point.z() + tl_height) / 2.0;
      const double max_distance_range = 200.0;
      if (!isInDistanceRange(tl_central_point, camera_pose.position, max_distance_range))
        continue;

      // check angle range
      const double tl_yaw = normalizeAngle(std::atan2(tl_right_down_point.y() - tl_left_down_point.y(), tl_right_down_point.x() - tl_left_down_point.x()) + M_PI_2);
      const double max_angele_range = 40.0 / 180.0 * M_PI;

      // get direction of z axis
      tf2::Vector3 camera_z_dir(0, 0, 1);
      tf2::Matrix3x3 camera_rotation_matrix(tf2::Quaternion(camera_pose.orientation.x, camera_pose.orientation.y,
                                                            camera_pose.orientation.z, camera_pose.orientation.w));
      camera_z_dir = camera_rotation_matrix * camera_z_dir;
      double camera_yaw = std::atan2(camera_z_dir.y(), camera_z_dir.x());
      camera_yaw = normalizeAngle(camera_yaw);
      if (!isInAngleRange(tl_yaw, camera_yaw, max_angele_range))
        continue;

      // check within image frame
      tf2::Transform tf_map2camera(tf2::Quaternion(camera_pose.orientation.x, camera_pose.orientation.y, camera_pose.orientation.z, camera_pose.orientation.w),
                                   tf2::Vector3(camera_pose.position.x, camera_pose.position.y, camera_pose.position.z));
      tf2::Transform tf_map2tl(tf2::Quaternion(0, 0, 0, 1),
                               tf2::Vector3(tl_central_point.x, tl_central_point.y, tl_central_point.z));
      tf2::Transform tf_camera2tl;
      tf_camera2tl = tf_map2camera.inverse() * tf_map2tl;

      geometry_msgs::Point camera2tl_point;
      camera2tl_point.x = tf_camera2tl.getOrigin().x();
      camera2tl_point.y = tf_camera2tl.getOrigin().y();
      camera2tl_point.z = tf_camera2tl.getOrigin().z();
      if (!isInImageFrame(camera_info, camera2tl_point))
        continue;

      visible_traffic_light.push_back(tl);
    }
  }
}

bool MapBasedDetector::isInDistanceRange(const geometry_msgs::Point &tl_point, const geometry_msgs::Point &camera_point, const double max_distance_range)
{
  const double sq_dist = (tl_point.x - camera_point.x) * (tl_point.x - camera_point.x) + (tl_point.y - camera_point.y) * (tl_point.y - camera_point.y);
  return (sq_dist < (max_distance_range * max_distance_range));
}

bool MapBasedDetector::isInAngleRange(const double &tl_yaw, const double &camera_yaw, const double max_angele_range)
{
  Eigen::Vector2d vec1, vec2;
  vec1 << std::cos(tl_yaw), std::sin(tl_yaw);
  vec2 << std::cos(camera_yaw), std::sin(camera_yaw);
  const double diff_angle = std::acos(vec1.dot(vec2));
  return (std::fabs(diff_angle) < max_angele_range);
}

bool MapBasedDetector::isInImageFrame(const sensor_msgs::CameraInfo &camera_info, const geometry_msgs::Point &point)
{
  const double &camera_x = point.x;
  const double &camera_y = point.y;
  const double &camera_z = point.z;
  const double &fx = camera_info.K[(0 * 3) + 0];
  const double &fy = camera_info.K[(1 * 3) + 1];
  const double &cx = camera_info.K[(0 * 3) + 2];
  const double &cy = camera_info.K[(1 * 3) + 2];
  if (camera_z <= 0.0)
    return false;
  const double image_u = (fx * camera_x + cx * camera_z) / camera_z;
  const double image_v = (fy * camera_y + cy * camera_z) / camera_z;
  if (0 <= image_u && image_u < camera_info.width)
    if (0 <= image_v && image_v < camera_info.height)
      return true;
  return false;
}

double MapBasedDetector::normalizeAngle(const double &angle)
{
  return std::atan2(std::cos(angle), std::sin(angle));
}

} // namespace traffic_light
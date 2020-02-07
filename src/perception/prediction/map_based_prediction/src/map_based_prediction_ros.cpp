/*
 * Copyright 2018-2019 Autoware Foundation. All rights reserved.
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

// headers in STL
#include <chrono>
#include <cmath>
#include <unordered_map>

#include <unique_id/unique_id.h>
#include <uuid_msgs/UniqueID.h>

//headers in ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <geometry_msgs/Pose.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


//lanelet
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>


// headers in local files
#include "map_based_prediction.h"
#include "map_based_prediction_ros.h"



//copy from mission planner code; could extract from utility function in the future
bool MapBasedPredictionROS::getClosestLanelet(
  const geometry_msgs::Pose& search_pose, 
  const lanelet::LaneletMapPtr& lanelet_map_ptr_,
  lanelet::Lanelet* closest_lanelet,
  double distance_thresh,
  std::string uuid_string)
{
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  lanelet::BasicPoint2d search_point(search_pose.position.x, search_pose.position.y);
  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
      lanelet::geometry::findNearest(lanelet_map_ptr_->laneletLayer, search_point, 3);
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  debug_accumulated_time_ += time.count()/(1000.0*1000.0);
  
  if (nearest_lanelets.empty())
  {
    ROS_ERROR("Failed to find the closest lane!");
    return false;
  }
  
  if(uuid2laneids_.size()==0 || 
     uuid2laneids_.count(uuid_string)==0)
  {
    if(nearest_lanelets.front().first < 0.1)
    {
      *closest_lanelet=nearest_lanelets.front().second;
      return true;
      
    }
  }
  else
  {
    for(const auto& laneid: uuid2laneids_.at(uuid_string))
    {
      for(const auto& lanelet: nearest_lanelets)
      {
        if(laneid == lanelet.second.id() && 
           lanelet.first < 0.1)
        {
          *closest_lanelet=lanelet.second;
          return true;
        }
      }
    }
  }
  
  //in case cannnot match with the uid, associate with nearest lanelet
  for(const auto& lanelet: nearest_lanelets)
  {
    if(lanelet.first < 0.1)
    {
      *closest_lanelet = nearest_lanelets.front().second;
      return true;
    }
  }
  return false;
}

double calculateDistance(
  const geometry_msgs::Point& point1,
  const geometry_msgs::Point& point2)
{
  double dx = point1.x - point2.x;
  double dy = point1.y - point2.y;
  double distance = std::sqrt(dx*dx+dy*dy);
  return distance;
}

//depricated in the future begin ----
bool MapBasedPredictionROS::getSelfPose(geometry_msgs::Pose &self_pose, const std_msgs::Header &header)
{
  try
  {
    geometry_msgs::TransformStamped transform;
    transform = tf_buffer_ptr_->lookupTransform(header.frame_id, "base_link",
                                           header.stamp, ros::Duration(0.1));
    self_pose.position.x = transform.transform.translation.x;
    self_pose.position.y = transform.transform.translation.y;
    self_pose.position.z = transform.transform.translation.z;
    self_pose.orientation.x = transform.transform.rotation.x;
    self_pose.orientation.y = transform.transform.rotation.y;
    self_pose.orientation.z = transform.transform.rotation.z;
    self_pose.orientation.w = transform.transform.rotation.w;
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    return false;
  }
}

bool MapBasedPredictionROS::getSelfPoseInMap(geometry_msgs::Pose &self_pose)
{
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time();
  return getSelfPose(self_pose, header);
}
//deprecated in the future end -----

MapBasedPredictionROS::MapBasedPredictionROS()
:
pnh_("~"),
interpolating_resolution_(0.5)
{
  tf_buffer_ptr_ = std::make_shared<tf2_ros::Buffer>();
  tf_listener_ptr_= std::make_shared<tf2_ros::TransformListener>(*tf_buffer_ptr_);
  pnh_.param<bool>("map_based_prediction/has_subscribed_map", has_subscribed_map_, false);
  pnh_.param<double>("prediction_time_horizon", prediction_time_horizon_, 10.0);
  pnh_.param<double>("prediction_sampling_delta_time", prediction_sampling_delta_time_, 0.5);
  map_based_prediction_ = std::make_shared<MapBasedPrediction>(interpolating_resolution_,prediction_time_horizon_, prediction_sampling_delta_time_);
  debug_ego_uid_= std::make_shared<uuid_msgs::UniqueID>
                    (unique_id::toMsg(unique_id::fromRandom()));
}

void MapBasedPredictionROS::createROSPubSub()
{
  sub_objects_ = nh_.subscribe<autoware_perception_msgs::DynamicObjectArray>
    ("/perception/tracking/objects", 1, &MapBasedPredictionROS::objectsCallback, this);
  sub_map_ = nh_.subscribe
   ("/lanelet_map_bin", 10, &MapBasedPredictionROS::mapCallback, this);
    
  pub_objects_ = nh_.advertise<autoware_perception_msgs::DynamicObjectArray>("objects", 1);
  pub_markers_ = nh_.advertise<visualization_msgs::MarkerArray>("objects_path_markers", 1);
}

void MapBasedPredictionROS::objectsCallback(
  const autoware_perception_msgs::DynamicObjectArrayConstPtr& in_objects)
{
  debug_accumulated_time_ = 0.0;
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  
  if(!lanelet_map_ptr_)
  {
    std::cout << "has not been subscribed to map " << std::endl;
    return;
  }
  
  geometry_msgs::TransformStamped world2map_transform;
  geometry_msgs::TransformStamped map2world_transform;
  geometry_msgs::TransformStamped debug_map2lidar_transform;
  try
  {
    world2map_transform = tf_buffer_ptr_->lookupTransform(
                                          "map", //target
                                          in_objects->header.frame_id, //src
                                          in_objects->header.stamp,
                                          ros::Duration(0.1));
    map2world_transform = tf_buffer_ptr_->lookupTransform(
                                          in_objects->header.frame_id, //target
                                          "map", //src
                                          in_objects->header.stamp,
                                          ros::Duration(0.1));
    debug_map2lidar_transform = tf_buffer_ptr_->lookupTransform(
                                        "base_link", //target
                                        "map", //src
                                        ros::Time(),
                                        ros::Duration(0.1));
  }
  catch (tf2::TransformException &ex)
  {
    return;
  }
  
  autoware_perception_msgs::DynamicObjectArray tmp_objects_whitout_map;
  tmp_objects_whitout_map.header = in_objects->header;
  DynamicObjectWithLanesArray prediction_input;
  prediction_input.header = in_objects->header;
  for(const auto& object: in_objects->objects)
  {
    DynamicObjectWithLanes tmp_object;
    tmp_object.object =object;
    if(in_objects->header.frame_id != "map")
    {
      geometry_msgs::Pose pose_in_map;
      tf2::doTransform(object.state.pose_covariance.pose,
                      pose_in_map,
                      world2map_transform);
      tmp_object.object.state.pose_covariance.pose = pose_in_map;
    }
    
    if(object.semantic.type != autoware_perception_msgs::Semantic::CAR&&
       object.semantic.type != autoware_perception_msgs::Semantic::BUS&&
       object.semantic.type != autoware_perception_msgs::Semantic::TRUCK)
    {
      tmp_objects_whitout_map.objects.push_back(tmp_object.object);
      continue;
    }
    
    if(tmp_object.object.state.twist_covariance.twist.linear.x< 0 && 
       tmp_object.object.state.orientation_reliable)
    {
      tmp_objects_whitout_map.objects.push_back(tmp_object.object);
      continue;
    }
    else if(tmp_object.object.state.twist_covariance.twist.linear.x < 0)
    {
      tmp_object.object.state.twist_covariance.twist.linear.x *= -1;
      double yaw = M_PI + 
                   tf2::getYaw(tmp_object.object.state.pose_covariance.pose.orientation);
      float roll = 0;
      float pitch = 0;
      tf2::Quaternion q;
      q.setRPY( roll, pitch, yaw );
      tmp_object.object.state.pose_covariance.pose.orientation = 
        tf2::toMsg(q);
    }
                     
                     
    lanelet::Lanelet start_lanelet;
    geometry_msgs::Point closest_point;
    std::vector<geometry_msgs::Pose> path_points;
    std::vector<geometry_msgs::Pose> second_path_points;
    std::vector<geometry_msgs::Pose> right_path_points;
    std::string uuid_string = unique_id::toHexString(*debug_ego_uid_);
    double epsilon = std::numeric_limits<double>::epsilon();
    if(!getClosestLanelet(tmp_object.object.state.pose_covariance.pose, 
                          lanelet_map_ptr_,
                          &start_lanelet,
                          epsilon,
                          uuid_string))
    {
      geometry_msgs::Point debug_point;
      tf2::doTransform(tmp_object.object.state.pose_covariance.pose.position,
                     debug_point,
                     debug_map2lidar_transform);
      tmp_objects_whitout_map.objects.push_back(object);
      continue;
    }
    
    auto opt_right = routing_graph_ptr_->right(start_lanelet);
    lanelet::routing::LaneletPaths right_paths;
    if (!!opt_right)
    {
      right_paths =  
        routing_graph_ptr_->possiblePaths(*opt_right, 
                                          150, 0,false);
    }
    auto opt_left = routing_graph_ptr_->left(start_lanelet);
    lanelet::routing::LaneletPaths left_paths;
    if (!!opt_left)
    {
      left_paths =  
        routing_graph_ptr_->possiblePaths(*opt_left, 
                                          150, 0,false);
    }
    
    lanelet::routing::LaneletPaths paths =  
      routing_graph_ptr_->possiblePaths(start_lanelet, 
                                        150, 0,false);
   
    paths.insert(paths.end(), right_paths.begin(), right_paths.end());
    paths.insert(paths.end(), left_paths.begin(), left_paths.end());
    if(paths.size() == 0)
    {
      geometry_msgs::Point debug_point;
      tf2::doTransform(tmp_object.object.state.pose_covariance.pose.position,
                     debug_point,
                     debug_map2lidar_transform);
      tmp_objects_whitout_map.objects.push_back(object);
      continue;
    }
    
    std::vector<int> lanelet_ids;
    for(const auto& lanelet: paths.front())
    {
      lanelet_ids.push_back(lanelet.id());
    }
    
    std::string uid_string = unique_id::toHexString(object.id);
    if(uuid2laneids_.count(uid_string)==0)
    {
      uuid2laneids_.emplace(uid_string, lanelet_ids);
    }
    else
    {
      uuid2laneids_.at(uid_string) = lanelet_ids;
    }
    
    geometry_msgs::Point debug_point;
    tf2::doTransform(tmp_object.object.state.pose_covariance.pose.position,
                     debug_point,
                     debug_map2lidar_transform);
    std::vector<std::vector<geometry_msgs::Pose>> tmp_paths;
    for(const auto& path: paths)
    {
      std::vector<geometry_msgs::Pose> tmp_path;
      for(const auto& lanelet: path)
      {
        for(const auto& point: lanelet.centerline())
        {
          geometry_msgs::Pose tmp_pose;
          tmp_pose.position.x = point.x();
          tmp_pose.position.y = point.y();
          tmp_pose.position.z = point.z();
          tmp_path.push_back(tmp_pose);
        }
      }
      tmp_paths.push_back(tmp_path);
    }
    tmp_object.lanes = tmp_paths;
    prediction_input.objects.push_back(tmp_object);
  }
  // std::cerr <<"lookup times: "<<in_objects->objects.size()<<", total lookup time: " <<debug_accumulated_time_<< " milli sec" << std::endl;
  
  // 1. 現在日時を取得
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  // std::cerr <<"prediction preprocess time " <<time.count()/(1000.0*1000.0)<< " milli sec" << std::endl;
  
  std::vector<autoware_perception_msgs::DynamicObject> out_objects_in_map;
  std::vector<geometry_msgs::Point> interpolated_points;
  map_based_prediction_->doPrediction(prediction_input, out_objects_in_map, interpolated_points);
  autoware_perception_msgs::DynamicObjectArray output;
  output.header = in_objects->header;
  output.header.frame_id = "map";
  output.objects = out_objects_in_map;
  
  std::vector<autoware_perception_msgs::DynamicObject> out_objects_without_map;
  map_based_prediction_->doLinearPrediction(tmp_objects_whitout_map, out_objects_without_map);
  output.objects.insert(output.objects.begin(), out_objects_without_map.begin(), out_objects_without_map.end());
  // for(const auto& object: out_objects_in_map)
  // {
  //   autoware_perception_msgs::DynamicObject tmp_object;
  //   tmp_object = object;
  //   geometry_msgs::Pose pose;
  //   tf2::doTransform(object.state.pose_covariance.pose,
  //                    pose,
  //                    map2world_transform);
  //   tmp_object.state.pose_covariance.pose = pose;
  //   for(auto& path: tmp_object.state.predicted_paths)
  //   {
  //     for(auto& point: path.path)
  //     {
  //       geometry_msgs::Pose tmp_pose;
  //       tf2::doTransform(point.pose.pose,
  //                        tmp_pose,
  //                        map2world_transform);
  //       point.pose.pose = tmp_pose;
  //     }
  //   }
  //   output.objects.push_back(tmp_object);
  // }
  pub_objects_.publish(output);
  
  // std::cerr << "-------------"  << std::endl;
  // visualization_msgs::MarkerArray marker_array;
  // int unique_id = 0;
  
  // // visualize cubic spline point
  // visualization_msgs::Marker debug_interpolated_points; 
  // debug_interpolated_points.lifetime = ros::Duration(1.0);
  // debug_interpolated_points.header.frame_id = "map";
  // debug_interpolated_points.header.stamp = in_objects->header.stamp;
  // debug_interpolated_points.ns = std::string("debug_interpolated_points");
  // debug_interpolated_points.action = visualization_msgs::Marker::MODIFY;
  // debug_interpolated_points.pose.orientation.w = 1.0;
  // debug_interpolated_points.id = unique_id;
  // debug_interpolated_points.type = visualization_msgs::Marker::SPHERE_LIST;
  // debug_interpolated_points.scale.x = 1.0f;
  // debug_interpolated_points.scale.y = 0.1f;
  // debug_interpolated_points.scale.z = 0.1f;
  // debug_interpolated_points.color.r = 1.0f;
  // debug_interpolated_points.color.a = 0.999;
  // for(const auto& point: interpolated_points)
  // {
  //   debug_interpolated_points.points.push_back(point);
  // }
  // unique_id++;
  // if(interpolated_points.size()>0)
  // {
  //   marker_array.markers.push_back(debug_interpolated_points);
  // }
  
  // // visualize cubic spline point
  // visualization_msgs::Marker debug_out_points;
  // debug_out_points.lifetime = ros::Duration(1.0);
  // debug_out_points.header.frame_id = "map";
  // debug_out_points.header.stamp = ros::Time();
  // debug_out_points.ns = std::string("debug_out_points");
  // debug_out_points.action = visualization_msgs::Marker::MODIFY;
  // debug_out_points.pose.orientation.w = 1.0;
  // debug_out_points.id = unique_id;
  // debug_out_points.type = visualization_msgs::Marker::SPHERE_LIST;
  // debug_out_points.scale.x = 1.0f;
  // debug_out_points.scale.y = 0.1f;
  // debug_out_points.scale.z = 0.1f;
  // debug_out_points.color.r = 1.0f;
  // debug_out_points.color.a = 0.999;
  // for(const auto& object: out_objects_in_map)
  // {
  //   for(const auto& path: object.state.predicted_paths)
  //   {
  //     for(const auto& point: path.path)
  //     {
  //       debug_out_points.points.push_back(point.pose.pose.position);
  //     }
  //   }
  // }
  // unique_id++;
  // if(debug_out_points.points.size()>0)
  // {
  //   marker_array.markers.push_back(debug_out_points);
  // }
  
  // pub_markers_.publish(marker_array);
}

void MapBasedPredictionROS::mapCallback(
  const autoware_lanelet2_msgs::MapBin& msg)
{
  ROS_INFO("Start loading lanelet");
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(msg, 
                                        lanelet_map_ptr_,
                                        &traffic_rules_ptr_, 
                                        &routing_graph_ptr_);
  ROS_INFO("Map is loaded");
}

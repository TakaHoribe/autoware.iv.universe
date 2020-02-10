
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>



#include <tf2/utils.h>

#include <memory>
#include <chrono>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


#include "eb_path_planner/horibe_interpolate.h"
#include "eb_path_planner/modify_reference_path.h"
#include "eb_path_planner/eb_path_smoother.h"
#include "eb_path_planner/node.hpp"

namespace tmp1
{
  
geometry_msgs::Point transformToRelativeCoordinate2D(
  const geometry_msgs::Point &point,
  const geometry_msgs::Pose &origin)
{
  // translation
  geometry_msgs::Point trans_p;
  trans_p.x = point.x - origin.position.x;
  trans_p.y = point.y - origin.position.y;

  // rotation (use inverse matrix of rotation)
  double yaw = tf2::getYaw(origin.orientation);

  geometry_msgs::Point res;
  res.x = (cos(yaw) * trans_p.x) + (sin(yaw) * trans_p.y);
  res.y = ((-1) * sin(yaw) * trans_p.x) + (cos(yaw) * trans_p.y);
  res.z = origin.position.z;

  return res;
}

bool transformMapToImage(const geometry_msgs::Point& map_point,
                         const nav_msgs::MapMetaData& occupancy_grid_info,
                         geometry_msgs::Point& image_point)
{
  geometry_msgs::Point relative_p = 
    transformToRelativeCoordinate2D(map_point, occupancy_grid_info.origin);
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = relative_p.x/resolution;
  double map_y_in_image_resolution = relative_p.y/resolution;
  double image_x = map_y_height - map_y_in_image_resolution;
  double image_y = map_x_width - map_x_in_image_resolution;
  if(image_x>=0 && 
     image_x<(int)map_y_height &&
     image_y>=0 && 
     image_y<(int)map_x_width)
  {
    image_point.x = image_x;
    image_point.y = image_y;
    return true;
  }
  else
  {
    return false;
  } 
}



bool transformImageToMap(const geometry_msgs::Point& image_point,
                         const nav_msgs::MapMetaData& occupancy_grid_info,
                         geometry_msgs::Point& map_point)
{
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = map_x_width - image_point.y;
  double map_y_in_image_resolution = map_y_height - image_point.x;
  double relative_x = map_x_in_image_resolution*resolution;
  double relative_y = map_y_in_image_resolution*resolution;

  double yaw = tf2::getYaw(occupancy_grid_info.origin.orientation);
  geometry_msgs::Point res;
  res.x = (cos(-yaw) * relative_x) + (sin(-yaw) * relative_y);
  res.y = ((-1) * sin(-yaw) * relative_x) + (cos(-yaw) * relative_y);
  
  map_point.x = res.x + occupancy_grid_info.origin.position.x;
  map_point.y = res.y + occupancy_grid_info.origin.position.y;
  map_point.z = occupancy_grid_info.origin.position.z;
  return true;
}
}

namespace path_planner
{
EBPathPlannerNode::EBPathPlannerNode(): 
nh_(),
private_nh_("~")
{
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("eb_path_planner_marker", 1, true);
  path_pub_ = 
    nh_.advertise<autoware_planning_msgs::Path>
      ("planning/scenario_planning/scenarios/lane_following/motion_planning/avoiding_path", 
        1, true);
  debug_traj_pub_ = 
   nh_.advertise<autoware_planning_msgs::Trajectory>("/debug_traj", 1, true);
  debug_fixed_traj_pub_ = 
   nh_.advertise<autoware_planning_msgs::Trajectory>("/debug_fixed_traj", 1, true);  
  debug_clearance_map_in_occupancy_grid_pub_ = 
   nh_.advertise<nav_msgs::OccupancyGrid>("debug_clearance_map", 1, true);
    
  
  
  // is_relay_path_sub_ = private_nh_.subscribe("/is_relay_path", 1, 
  //                          &EBPathPlannerNode::isRelayPathCallback, this);
  twist_sub_ = private_nh_.subscribe("/current_velocity", 1, 
                           &EBPathPlannerNode::currentVelocityCallback, this);
  objects_sub_ = private_nh_.subscribe("/perception/prediction/objects", 10,
                           &EBPathPlannerNode::objectsCallback, this);
  private_nh_.param<bool>("is_debug_clearance_map_mode", 
                           is_debug_clearance_map_mode_,true);
  private_nh_.param<bool>("is_debug_drivable_area_mode", 
                           is_debug_drivable_area_mode_,false);
  private_nh_.param<bool>("is_publishing_clearance_map_as_occupancy_grid", 
                           is_publishing_clearance_map_as_occupancy_grid_,false);
  private_nh_.param<int>("number_of_backward_detection_range_path_points", 
                             number_of_backward_detection_range_path_points_, 5);
  private_nh_.param<double>("forward_fixing_distance", 
                             forward_fixing_distance_, 20.0);
  private_nh_.param<double>("backward_fixing_distance", 
                             backward_fixing_distance_, 20.0);
  private_nh_.param<double>("detection_radius_from_ego", 
                             detection_radius_from_ego_, 50.0);
  private_nh_.param<double>("detection_radius_around_path_point", 
                             detection_radius_around_path_point_, 2.5);
  private_nh_.param<double>("reset_delta_ego_distance", 
                             reset_delta_ego_distance_, 5.0);
  private_nh_.param<double>("exploring_minumum_radius", 
                             exploring_minimum_radius_, 1.3);
  private_nh_.param<double>("delta_arc_length_for_path_smoothing", 
                             delta_arc_length_for_path_smoothing_, 1.0);
  private_nh_.param<double>("delta_arc_length_for_explored_points", 
                             delta_arc_length_for_explored_points_, 0.7);
  private_nh_.param<double>("max_avoiding_objects_velocity_ms", 
                             max_avoiding_objects_velocity_ms_, 0.1);
  private_nh_.param<double>("clearance_weight_when_exploring", 
                             clearance_weight_when_exploring_, 0.0);
  is_previously_avoidance_mode_ = false;
  doResetting();
}

EBPathPlannerNode::~EBPathPlannerNode() {}

bool EBPathPlannerNode::isAvoidanceNeeded(
  const std::vector<autoware_planning_msgs::PathPoint> in_path,
  const geometry_msgs::Pose self_pose,
  const std::vector<autoware_planning_msgs::TrajectoryPoint>& previous_output_trajectory_points) 
{
  
  bool is_objects_detected = 
    detectAvoidingObjectsOnPath(
      self_pose, 
      in_objects_ptr_->objects,
      in_path);
  
  if (is_objects_detected)
  {
    return true;
  } 
  
  if (!is_previously_avoidance_mode_)
  {
    // ROS_WARN("previous mode was not avoidance && does not detect objects on path; relay path");
    return false;
  }
  
  geometry_msgs::Pose nearest_pose;
  if(!getNearestPose(self_pose, previous_output_trajectory_points, nearest_pose))
  {
    return false;
  }
  
  return isPoseCloseToPath(in_path, nearest_pose) ? false : true;
}

bool EBPathPlannerNode::getNearestPose(
  const geometry_msgs::Pose self_pose,
  const std::vector<autoware_planning_msgs::TrajectoryPoint>& trajectory_points,
  geometry_msgs::Pose& nearest_pose) 
{
  double min_dist = 999999999;
  geometry_msgs::Pose tmp_nearest_pose;
  for(const auto& point: trajectory_points)
  {
    double dx = point.pose.position.x - self_pose.position.x;
    double dy = point.pose.position.y - self_pose.position.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if(dist < min_dist)
    {
      min_dist = dist;
      tmp_nearest_pose = point.pose;
    }
  }
  double thres = 3;
  if(min_dist < thres)
  {
    nearest_pose = tmp_nearest_pose;
    return true;
  }
  else
  {
    return false;
  }
}

bool EBPathPlannerNode::isPoseCloseToPath(
  const std::vector<autoware_planning_msgs::PathPoint> in_path,
  const geometry_msgs::Pose in_pose) 
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(size_t i = 0; i <  in_path.size(); i++)
  {
    tmp_x.push_back(in_path[i].pose.position.x);
    tmp_y.push_back(in_path[i].pose.position.y);
  }
  
  std::vector<double> base_s = horibe_spline::calcEuclidDist(tmp_x, tmp_y);
  std::vector<double> new_s;
  double interval_dist = 0.1;
  for(double i = 0.0; 
      i <= base_s.back();
      i += interval_dist)
  {
    new_s.push_back(i);
  }
  horibe_spline::SplineInterpolate spline;
  std::vector<double> new_x;
  std::vector<double> new_y;
  spline.interpolate(base_s, tmp_x, new_s, new_x);
  spline.interpolate(base_s, tmp_y, new_s, new_y);
  int min_ind = -1;
  double min_dist = 99999999;
  for (int i = 0; i < new_x.size(); i++)
  {
    double dx = new_x[i] - in_pose.position.x;
    double dy = new_y[i] - in_pose.position.y;
    double dist = (dx*dx+dy*dy);
    if(dist < min_dist)
    {
      min_dist = dist;
      min_ind = i;
    }
  }
  if(min_ind == -1)
  {
    return false;
  }
  double path_yaw;
  if(min_ind > 0)
  {
    double dx1 = new_x[min_ind] - new_x[min_ind-1];
    double dy1 = new_y[min_ind] - new_y[min_ind-1];
    path_yaw = std::atan2(dy1, dx1);
  }
  else
  {
    double dx1 = new_x[min_ind+1] - new_x[min_ind];
    double dy1 = new_y[min_ind+1] - new_y[min_ind];
    path_yaw = std::atan2(dy1, dx1);
  }
  double traj_yaw = tf2::getYaw(in_pose.orientation); 
  double delta_yaw = std::fabs(traj_yaw - path_yaw);
  double cos_similarity = std::cos(traj_yaw)*std::cos(path_yaw) +
                          std::sin(traj_yaw)*std::sin(traj_yaw);
  double distance_threshold = 0.2;
  double cos_similarity_threshold = 0.9;
  if(min_dist < distance_threshold && 
     cos_similarity > cos_similarity_threshold)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void EBPathPlannerNode::callback(
  const autoware_planning_msgs::Path &input_path_msg,
  autoware_planning_msgs::Trajectory &output_trajectory_msg)                               
{
  
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  if(!in_objects_ptr_)
  {
    in_objects_ptr_ = std::make_unique<autoware_perception_msgs::DynamicObjectArray>();
  }
  geometry_msgs::Pose self_pose;
  if (!getSelfPoseInMap(self_pose))
  {
    return;
  }
  if(!previous_ego_point_ptr_)
  {
    previous_ego_point_ptr_ = std::make_unique<geometry_msgs::Point>(self_pose.position);
    return;
  }
  
  if (!isAvoidanceNeeded(
      input_path_msg.points, 
      self_pose,
      *previous_optimized_points_ptr_))
  {
    generateSmoothTrajectory(
      self_pose,
      input_path_msg,
      output_trajectory_msg);
    is_previously_avoidance_mode_ = false;
    doResetting();
  }
  else
  {
    cv::Mat clearance_map;
    generateClearanceMap(
      input_path_msg.drivable_area,
      in_objects_ptr_->objects,
      self_pose,
      clearance_map);
    
    if(needReset(self_pose.position,
                 previous_ego_point_ptr_,
                 clearance_map,
                 input_path_msg.drivable_area.info,
                 *previous_explored_points_ptr_,
                 in_objects_ptr_->objects))
    {
      doResetting();
    }
    
    std::cout << "prev ex point size "<< previous_explored_points_ptr_->size() << std::endl;
    geometry_msgs::Point start_exploring_point;
    geometry_msgs::Point goal_exploring_point;
    std::vector<geometry_msgs::Point> truncated_explored_points;
    if(needExprolation(
        self_pose.position,
        *previous_explored_points_ptr_,
        input_path_msg,
        clearance_map,
        start_exploring_point,
        goal_exploring_point,
        truncated_explored_points))
    {
      visualization_msgs::MarkerArray marker_array;
      int unique_id = 0;
      visualization_msgs::Marker start_marker;
      start_marker.lifetime = ros::Duration(2.0);
      start_marker.header = input_path_msg.header;
      start_marker.ns = std::string("start_point");
      start_marker.action = visualization_msgs::Marker::MODIFY;
      start_marker.pose.orientation.w = 1.0;
      start_marker.pose.position = start_exploring_point;
      start_marker.id = unique_id;
      start_marker.type = visualization_msgs::Marker::SPHERE;
      start_marker.scale.x = 1.0f;
      start_marker.scale.y = 1.0f;
      start_marker.scale.z = 1.0f;
      start_marker.color.r = 1.0f;
      start_marker.color.a = 1.0;
      unique_id++;
      marker_array.markers.push_back(start_marker);
      
      visualization_msgs::Marker goal_marker;
      goal_marker.lifetime = ros::Duration(2.0);
      goal_marker.header = input_path_msg.header;
      goal_marker.ns = std::string("goal_point");
      goal_marker.action = visualization_msgs::Marker::MODIFY;
      goal_marker.pose.orientation.w = 1.0;
      goal_marker.pose.position = goal_exploring_point;
      goal_marker.id = unique_id;
      goal_marker.type = visualization_msgs::Marker::SPHERE;
      goal_marker.scale.x = 1.0f;
      goal_marker.scale.y = 1.0f;
      goal_marker.scale.z = 1.0f;
      goal_marker.color.r = 1.0f;
      goal_marker.color.a = 1.0;
      unique_id++;
      marker_array.markers.push_back(goal_marker);
      markers_pub_.publish(marker_array);
      
      
      std::vector<geometry_msgs::Point> explored_points;
      bool is_explore_success = 
        modify_reference_path_ptr_->generateModifiedPath(
              self_pose, 
              start_exploring_point,
              goal_exploring_point,
              input_path_msg.points,
              in_objects_ptr_->objects,
              explored_points,
              clearance_map, 
              input_path_msg.drivable_area.info);
      if(!is_explore_success)
      {
        // ROS_WARN("explore fail, relay path");
        ROS_WARN_THROTTLE(3.0,
          "[EBPathPlanner] Could not find avoiging path; relay path");
        generateSmoothTrajectory(
          self_pose,
          input_path_msg,
          output_trajectory_msg);
        is_previously_avoidance_mode_ = false;
        previous_ego_point_ptr_ = 
          std::make_unique<geometry_msgs::Point>(self_pose.position);
        previous_optimized_points_ptr_ = 
          std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>
            (output_trajectory_msg.points);
        return;
      }
      
      //merge truncate with explored point
      for (int i = 0; i < explored_points.size(); i++)
      {
        truncated_explored_points.push_back(explored_points[i]);
      }
      // remove redundant explored points 
      std::vector<geometry_msgs::Point> non_redundant_explored_points;
      for (int i = 0; i < truncated_explored_points.size(); i++)
      {
        if(i>0)
        {
          double dx = truncated_explored_points[i].x -truncated_explored_points[i-1].x;
          double dy = truncated_explored_points[i].y -truncated_explored_points[i-1].y;
          double dist = std::sqrt(dx*dx+dy*dy);
          if(dist < 1e-6)
          {
            continue;
          }
        }
        non_redundant_explored_points.push_back(truncated_explored_points[i]);
      }
      previous_explored_points_ptr_ = 
        std::make_unique<std::vector<geometry_msgs::Point>>(non_redundant_explored_points);    
    }
    
    
    std::vector<geometry_msgs::Point> debug_interpolated_points;
    std::vector<geometry_msgs::Point> debug_constrain_points;
    std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
    eb_path_smoother_ptr_->generateOptimizedExploredPoints(
          input_path_msg.points,
          *previous_explored_points_ptr_, 
          self_pose,
          clearance_map, 
          input_path_msg.drivable_area.info,
          debug_interpolated_points,
          debug_constrain_points,
          optimized_points);
    
    alighWithPathPoints(
      input_path_msg.points,
      optimized_points);
    std::vector<autoware_planning_msgs::TrajectoryPoint> fine_optimized_points;
    generateFineOptimizedPoints(
      input_path_msg.points,
      optimized_points, 
      fine_optimized_points);
    output_trajectory_msg.points = fine_optimized_points;
    output_trajectory_msg.header = input_path_msg.header;
    is_previously_avoidance_mode_ = true;
    
    std::chrono::high_resolution_clock::time_point end= 
      std::chrono::high_resolution_clock::now();
    std::chrono::nanoseconds time = 
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
    std::cout << "    total time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;

    std::cout << "-------------------" << std::endl;
    //debug; marker array
    visualization_msgs::MarkerArray marker_array;
    int unique_id = 0;
    // visualize cubic spline point
    visualization_msgs::Marker debug_interpolated_points_marker;
    debug_interpolated_points_marker.lifetime = ros::Duration(0.1);
    debug_interpolated_points_marker.header = input_path_msg.header;
    debug_interpolated_points_marker.ns = std::string("interpolated_points_marker");
    debug_interpolated_points_marker.action = visualization_msgs::Marker::MODIFY;
    debug_interpolated_points_marker.pose.orientation.w = 1.0;
    debug_interpolated_points_marker.id = unique_id;
    debug_interpolated_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_interpolated_points_marker.scale.x = 0.8f;
    debug_interpolated_points_marker.scale.y = 0.1f;
    debug_interpolated_points_marker.scale.z = 0.1f;
    debug_interpolated_points_marker.color.g = 1.0f;
    debug_interpolated_points_marker.color.a = 0.699;
    for(const auto& point: debug_interpolated_points)
    {
      debug_interpolated_points_marker.points.push_back(point);
    }
    if(!debug_interpolated_points_marker.points.empty())
    {
      marker_array.markers.push_back(debug_interpolated_points_marker);
    }
    unique_id++;
    
    // visualize cubic spline point
    visualization_msgs::Marker debug_cubic_spline;
    debug_cubic_spline.lifetime = ros::Duration(0.1);
    debug_cubic_spline.header = input_path_msg.header;
    debug_cubic_spline.ns = std::string("explored_points");
    debug_cubic_spline.action = visualization_msgs::Marker::MODIFY;
    debug_cubic_spline.pose.orientation.w = 1.0;
    debug_cubic_spline.id = unique_id;
    debug_cubic_spline.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_cubic_spline.scale.x = 1.0f;
    debug_cubic_spline.scale.y = 0.1f;
    debug_cubic_spline.scale.z = 0.1f;
    debug_cubic_spline.color.g = 1.0f;
    debug_cubic_spline.color.a = 0.999;
    if(previous_explored_points_ptr_)
    {
      for(const auto& point: *previous_explored_points_ptr_)
      {
        debug_cubic_spline.points.push_back(point);
      }
      if(!debug_cubic_spline.points.empty())
      {
        marker_array.markers.push_back(debug_cubic_spline);
      }
    }
    unique_id++;
    // visualize cubic spline point
    visualization_msgs::Marker debug_optimized_points_marker;
    debug_optimized_points_marker.lifetime = ros::Duration(0.1);
    debug_optimized_points_marker.header = input_path_msg.header;
    debug_optimized_points_marker.ns = std::string("optimized_points_marker");
    debug_optimized_points_marker.action = visualization_msgs::Marker::MODIFY;
    debug_optimized_points_marker.pose.orientation.w = 1.0;
    debug_optimized_points_marker.id = unique_id;
    debug_optimized_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_optimized_points_marker.scale.x = 1.0f;
    debug_optimized_points_marker.scale.y = 0.1f;
    debug_optimized_points_marker.scale.z = 0.1f;
    debug_optimized_points_marker.color.g = 1.0f;
    debug_optimized_points_marker.color.a = 0.999;
    for (size_t i = 0; i < optimized_points.size(); i++)
    {
      debug_optimized_points_marker.points.push_back(optimized_points[i].pose.position);
    }
    if(!debug_optimized_points_marker.points.empty())
    {
      marker_array.markers.push_back(debug_optimized_points_marker);
    }
    unique_id++;
    
    for (size_t i = 0; i < output_trajectory_msg.points.size(); i++)
    {
      // visualize cubic spline point
      visualization_msgs::Marker debug_optimized_points_marker;
      debug_optimized_points_marker.lifetime = ros::Duration(0.1);
      debug_optimized_points_marker.header = input_path_msg.header;
      debug_optimized_points_marker.ns = std::string("optimized_points_arow_marker");
      debug_optimized_points_marker.action = visualization_msgs::Marker::MODIFY;
      debug_optimized_points_marker.pose= output_trajectory_msg.points[i].pose;
      debug_optimized_points_marker.id = unique_id;
      debug_optimized_points_marker.type = visualization_msgs::Marker::ARROW;
      debug_optimized_points_marker.scale.x = 0.3f;
      debug_optimized_points_marker.scale.y = 0.1f;
      debug_optimized_points_marker.scale.z = 0.1f;
      debug_optimized_points_marker.color.g = 1.0f;
      debug_optimized_points_marker.color.a = 0.999;
      unique_id++;
      marker_array.markers.push_back(debug_optimized_points_marker);
    }
    
    for (int i = 0; i < debug_constrain_points.size(); i++)
    {
      visualization_msgs::Marker constrain_points_marker;
      constrain_points_marker.lifetime = ros::Duration(0.1);
      constrain_points_marker.header = input_path_msg.header;
      constrain_points_marker.ns = std::string("constrain_points_marker");
      constrain_points_marker.action = visualization_msgs::Marker::MODIFY;
      constrain_points_marker.pose.orientation.w = 1.0;
      constrain_points_marker.pose.position = debug_constrain_points[i];
      constrain_points_marker.pose.position.z = self_pose.position.z;
      constrain_points_marker.id = unique_id;
      constrain_points_marker.type = visualization_msgs::Marker::SPHERE;
      constrain_points_marker.scale.x = std::abs(debug_constrain_points[i].z*2);
      constrain_points_marker.scale.y = std::abs(debug_constrain_points[i].z*2);
      constrain_points_marker.scale.z = 0.1;
      constrain_points_marker.color.r = 1.0f;
      constrain_points_marker.color.g = 0.5f;
      constrain_points_marker.color.a = 0.199;
      unique_id++;
      marker_array.markers.push_back(constrain_points_marker);
    }
          
    for (size_t i = 0; i < optimized_points.size(); i++)
    {
      visualization_msgs::Marker text_marker;
      text_marker.lifetime = ros::Duration(0.1);
      text_marker.header = input_path_msg.header;
      text_marker.ns = std::string("text_optimized_points");
      text_marker.action = visualization_msgs::Marker::MODIFY;
      text_marker.pose.orientation.w = 1.0;
      text_marker.id = unique_id;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.scale.z = 0.1f;
      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.a = 0.999;
      text_marker.text = std::to_string((int)i);
      text_marker.pose.position = optimized_points[i].pose.position;
      text_marker.pose.orientation.w = 1.0;
      marker_array.markers.push_back(text_marker);
      unique_id++;
    }
    
    markers_pub_.publish(marker_array);
  }
  previous_ego_point_ptr_ = std::make_unique<geometry_msgs::Point>(self_pose.position);
  previous_optimized_points_ptr_ = 
    std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>
      (output_trajectory_msg.points);
}

void EBPathPlannerNode::doResetting()
{
  modify_reference_path_ptr_ = std::make_unique<ModifyReferencePath>(
                  exploring_minimum_radius_,
                  backward_fixing_distance_,
                  clearance_weight_when_exploring_);
  eb_path_smoother_ptr_ = std::make_unique<EBPathSmoother>(
              exploring_minimum_radius_,
              backward_fixing_distance_,
              forward_fixing_distance_,
              delta_arc_length_for_path_smoothing_,
              delta_arc_length_for_explored_points_);
  previous_explored_points_ptr_ = 
    std::make_unique<std::vector<geometry_msgs::Point>>();
  // previous_optimized_points_ptr_ = 
  //   std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>();
}

void EBPathPlannerNode::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  in_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(msg);
}

// void EBPathPlannerNode::isRelayPathCallback(const std_msgs::Bool& msg)
// {
//   is_relaying_path_mode_ = msg.data;
// }

void EBPathPlannerNode::objectsCallback(
  const autoware_perception_msgs::DynamicObjectArray& msg)
{
  in_objects_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(msg);
}

bool EBPathPlannerNode::needReset(
  const geometry_msgs::Point& current_ego_point,
  const std::unique_ptr<geometry_msgs::Point>& previous_ego_point_ptr,
  const cv::Mat& clearance_map,
  const nav_msgs::MapMetaData& map_info,
  const std::vector<geometry_msgs::Point>& fixed_explored_points,
  const std::vector<autoware_perception_msgs::DynamicObject>& objects)
{
  bool is_need_reset = false;
  //check1
  double dx = current_ego_point.x - previous_ego_point_ptr->x;
  double dy = current_ego_point.y - previous_ego_point_ptr->y;
  double dist = std::sqrt(dx*dx+dy*dy);
  // previous_ego_point_ptr = std::make_unique<geometry_msgs::Point>(current_ego_point);
  if(dist > reset_delta_ego_distance_)
  {
    ROS_WARN_THROTTLE(3.0,
    "[EBPathPlanner] Reset eb path planner since delta ego distance is more than %lf",
     reset_delta_ego_distance_);
    is_need_reset = true;
    return is_need_reset;
  }
  
  //check2
  geometry_msgs::Point ego_point_in_image;
  bool is_inside_map = tmp1::transformMapToImage(
          current_ego_point, 
          map_info,
          ego_point_in_image);
  
  float clearance = 
    clearance_map.ptr<float>((int)ego_point_in_image.y)
                            [(int)ego_point_in_image.x]*map_info.resolution;
  if(clearance < 1e-6 || !is_inside_map)
  {
    ROS_WARN_THROTTLE(3.0,
    "[EBPathPlanner] Reset eb path planner since current ego vehicle is outside of drivavle area");
    is_need_reset = true;
    return is_need_reset;
  }
  
  
  //check3
  bool is_object_on_explored_points =  
    detectAvoidingObjectsOnPoints(
      objects,
      fixed_explored_points);
  if(is_object_on_explored_points)
  {
     ROS_WARN(
    "[EBPathPlanner] Reset eb path planner since objects on explored points");
    
    is_need_reset = true;
    return is_need_reset;
  }
  
  // //check4
  // if(is_relaying_path_mode_)
  // {
  //   ROS_WARN(
  //   "[EBPathPlanner] Reset eb path planner since is_relay_path_mode is enabled");
  //   is_need_reset = true;
  // }
  return is_need_reset;
}

bool EBPathPlannerNode::needExprolation(
  const geometry_msgs::Point& ego_point,
  const std::vector<geometry_msgs::Point>& previous_explored_points,
  const autoware_planning_msgs::Path& in_path,
  const cv::Mat& clearance_map,
  geometry_msgs::Point& start_exploring_point,
  geometry_msgs::Point& goal_exploring_point,
  std::vector<geometry_msgs::Point>& truncated_explored_points)
{
  double min_dist = 8888888;
  int min_ind = -1;
  for (int i = 0; i < previous_explored_points.size(); i++)
  {
    double dx = previous_explored_points[i].x - ego_point.x;
    double dy = previous_explored_points[i].y - ego_point.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_ind = i;
    }
  }
  
  double accum_dist = 0;
  for (int i = min_ind; i < previous_explored_points.size(); i++)
  {
    if(i>min_ind)
    {
      double dx = previous_explored_points[i].x - previous_explored_points[i-1].x;
      double dy = previous_explored_points[i].y - previous_explored_points[i-1].y;
      accum_dist += std::sqrt(dx*dx+dy*dy);
    }
  }
  if(accum_dist > 40)
  {
    return false;
  }
  
  std::cout << "need extend" << std::endl;
  truncated_explored_points = 
    truncateExploredPointsWithEgoVehicle
        (ego_point,
         previous_explored_points);
  std::cout << "truncate size "<<truncated_explored_points.size() << std::endl;
  if(truncated_explored_points.empty())
  {
    double min_dist = 9999999999;
    int min_ind = 0;
    for (int i = 0; i < in_path.points.size(); i++)
    {
      double dx = in_path.points[i].pose.position.x - ego_point.x;
      double dy = in_path.points[i].pose.position.y - ego_point.y;
      double dist = std::sqrt(dx*dx+dy*dy);
      if(dist < min_dist)
      {
        min_dist = dist; 
        min_ind = i;
      }
    }
    //assuming delta_arc_length in path is about 1m
    const double delta_arc_length_for_path = 1;
    const double backward_distace_for_exploration = 5;
    int start_exploring_ind = std::max(
      (int)(min_ind - backward_distace_for_exploration/delta_arc_length_for_path ),
      0);
    start_exploring_point = in_path.points[start_exploring_ind].pose.position;
  }
  else
  {
    start_exploring_point = truncated_explored_points.back();
  }
  int nearest_path_point_ind_from_start_exploring_point = 0;
  double min_dist2 = 999999999;
  for (int i = 0; i < in_path.points.size(); i++)
  {
    double dx = in_path.points[i].pose.position.x - 
                start_exploring_point.x;
    double dy = in_path.points[i].pose.position.y - 
                start_exploring_point.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if(dist < min_dist2)
    {
      min_dist2 = dist;
      nearest_path_point_ind_from_start_exploring_point = i;
    }
  }
  std::unique_ptr<geometry_msgs::Pose> exploring_goal_pose_in_map_ptr;
  double accum_dist2 = 0;
  for (int i = nearest_path_point_ind_from_start_exploring_point; 
           i < in_path.points.size(); i++)
  {
    if(in_path.points[i].twist.linear.x < 1e-6 &&
       i == in_path.points.size()-1 && 
       !truncated_explored_points.empty())
    {
      double dx =  in_path.points[i].pose.position.x - truncated_explored_points.back().x;
      double dy =  in_path.points[i].pose.position.y - truncated_explored_points.back().y;
      double dist = std::sqrt(dx*dx+dy*dy);
      if(dist < 1e-6)
      {
        // ROS_WARN_THROTTLE(10.0,"prevent redundant goal");
        // ROS_WARN("prevent redundant goal");
        return false;
      }
      exploring_goal_pose_in_map_ptr = 
            std::make_unique<geometry_msgs::Pose>(in_path.points[i].pose);
      break;
    }
    if(i>nearest_path_point_ind_from_start_exploring_point)
    {
      double dx = in_path.points[i].pose.position.x - 
                  in_path.points[i-1].pose.position.x;
      double dy = in_path.points[i].pose.position.y - 
                  in_path.points[i-1].pose.position.y;
      accum_dist2+= std::sqrt(dx*dx+dy*dy); 
    }
    geometry_msgs::Point image_point;
    if(tmp1::transformMapToImage(
        in_path.points[i].pose.position, 
        in_path.drivable_area.info,
        image_point))
    {
      int pixel_x = image_point.x;
      int pixel_y = image_point.y;
      float clearance = clearance_map.ptr<float>((int)pixel_y)[(int)pixel_x]; 
      if(clearance*in_path.drivable_area.info.resolution
          >=exploring_minimum_radius_)
      {
        if(accum_dist2 > exploring_minimum_radius_*15)
        {
          exploring_goal_pose_in_map_ptr = 
            std::make_unique<geometry_msgs::Pose>(in_path.points[i].pose);
        }
        if(accum_dist2 > exploring_minimum_radius_*20)
        {
          break;
        }
      }
    }
    else
    {
      break;
    }
  }
  
  if(!exploring_goal_pose_in_map_ptr)
  {
    ROS_WARN_THROTTLE(3.0, "[EBPathPlanner] Could not find appropriate goal");
    // ROS_WARN( "[EBPathPlanner] Could not find appropriate goal");
    return false;
  }
  else
  {
    goal_exploring_point = exploring_goal_pose_in_map_ptr->position;
    return true;
  } 
}

std::vector<geometry_msgs::Point> 
  EBPathPlannerNode::truncateExploredPointsWithEgoVehicle(
    const geometry_msgs::Point& ego_point,
    const std::vector<geometry_msgs::Point>& explored_points)
{
  double min_dist = 8888888;
  int min_ind = -1;
  for (int i = 0; i < explored_points.size(); i++)
  {
    double dx = explored_points[i].x - ego_point.x;
    double dy = explored_points[i].y - ego_point.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if (dist < min_dist)
    {
      min_dist = dist;
      min_ind = i;
    }
  }
  int backward_fixing_distance_for_truncating = backward_fixing_distance_*0.5;
  int start_ind = 
    std::max((int)(min_ind-
                  backward_fixing_distance_for_truncating/exploring_minimum_radius_), 0);
  std::vector<geometry_msgs::Point> truncated_explored_points;
  for (int i = start_ind; i < explored_points.size(); i++)
  {
    truncated_explored_points.push_back(explored_points[i]);
  }
  return truncated_explored_points;
}

bool EBPathPlannerNode::generateSmoothTrajectory(
  const geometry_msgs::Pose& ego_pose,
  const autoware_planning_msgs::Path& input_path,
  autoware_planning_msgs::Trajectory& output_trajectory)
{
  std::vector<geometry_msgs::Point> debug_constrain_points;
  std::vector<geometry_msgs::Point> debug_interpolated_points_used_for_optimization;
  std::vector<autoware_planning_msgs::TrajectoryPoint> smooth_trajectory_points;
  convertPathToSmoothTrajectory(
    ego_pose,
    input_path.points,
    smooth_trajectory_points,
    debug_constrain_points,
    debug_interpolated_points_used_for_optimization);
  alighWithPathPoints(
    input_path.points,
    smooth_trajectory_points);
  std::vector<autoware_planning_msgs::TrajectoryPoint> fine_optimized_points;
  generateFineOptimizedPoints(
    input_path.points,
    smooth_trajectory_points,
    fine_optimized_points);
  output_trajectory.header = input_path.header;
  output_trajectory.points = fine_optimized_points;
  
  
  visualization_msgs::MarkerArray marker_array;
  int unique_id = 0;
  visualization_msgs::Marker smooth_interpolated_points;
  smooth_interpolated_points.lifetime = ros::Duration(0.1);
  smooth_interpolated_points.header = input_path.header;
  smooth_interpolated_points.ns = std::string("smooth_interpolated_points");
  smooth_interpolated_points.action = visualization_msgs::Marker::MODIFY;
  smooth_interpolated_points.pose.orientation.w = 1.0;
  smooth_interpolated_points.id = unique_id;
  smooth_interpolated_points.type = visualization_msgs::Marker::SPHERE_LIST;
  smooth_interpolated_points.scale.x = 0.5f;
  smooth_interpolated_points.scale.y = 0.1f;
  smooth_interpolated_points.scale.z = 0.1f;
  smooth_interpolated_points.color.r = 1.0f;
  smooth_interpolated_points.color.g = 1.0f;
  smooth_interpolated_points.color.a = 0.999;
  for (int i = 0; i < debug_interpolated_points_used_for_optimization.size(); i++)
  {
    smooth_interpolated_points.points.push_back(debug_interpolated_points_used_for_optimization[i]);
  }
  if(!smooth_interpolated_points.points.empty())
  {
    marker_array.markers.push_back(smooth_interpolated_points);
  }
  unique_id++;
  for (int i = 0; i < debug_constrain_points.size(); i++)
  {
    visualization_msgs::Marker constrain_points_marker;
    constrain_points_marker.lifetime = ros::Duration(0.1);
    constrain_points_marker.header = input_path.header;
    constrain_points_marker.ns = std::string("constrain_points_marker");
    constrain_points_marker.action = visualization_msgs::Marker::MODIFY;
    constrain_points_marker.pose.orientation.w = 1.0;
    constrain_points_marker.pose.position = debug_constrain_points[i];
    constrain_points_marker.pose.position.z = ego_pose.position.z;
    constrain_points_marker.id = unique_id;
    constrain_points_marker.type = visualization_msgs::Marker::SPHERE;
    constrain_points_marker.scale.x = std::abs(debug_constrain_points[i].z*2);
    constrain_points_marker.scale.y = std::abs(debug_constrain_points[i].z*2);
    constrain_points_marker.scale.z = 0.1;
    constrain_points_marker.color.r = 1.0f;
    constrain_points_marker.color.g = 0.5f;
    constrain_points_marker.color.a = 0.199;
    unique_id++;
    marker_array.markers.push_back(constrain_points_marker);
  }
  
  visualization_msgs::Marker smooth_optimized_poitns_marker;
  smooth_optimized_poitns_marker.lifetime = ros::Duration(0.1);
  smooth_optimized_poitns_marker.header = input_path.header;
  smooth_optimized_poitns_marker.ns = std::string("smooth_optimized_poitns_marker");
  smooth_optimized_poitns_marker.action = visualization_msgs::Marker::MODIFY;
  smooth_optimized_poitns_marker.pose.orientation.w = 1.0;
  smooth_optimized_poitns_marker.id = unique_id;
  smooth_optimized_poitns_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  smooth_optimized_poitns_marker.scale.x = 0.5f;
  smooth_optimized_poitns_marker.scale.y = 0.1f;
  smooth_optimized_poitns_marker.scale.z = 0.1f;
  smooth_optimized_poitns_marker.color.r = 1.0f;
  smooth_optimized_poitns_marker.color.g = 1.0f;
  smooth_optimized_poitns_marker.color.a = 0.999;
  for (int i = 0; i < smooth_trajectory_points.size(); i++)
  {
    smooth_optimized_poitns_marker.points.push_back(
      smooth_trajectory_points[i].pose.position);
  }
  if(!smooth_optimized_poitns_marker.points.empty())
  {
    marker_array.markers.push_back(smooth_optimized_poitns_marker);
  }
  unique_id++;
  markers_pub_.publish(marker_array);
}

bool EBPathPlannerNode::detectAvoidingObjectsOnPath(
  const geometry_msgs::Pose& ego_pose,
  const std::vector<autoware_perception_msgs::DynamicObject>& objects,
  const std::vector<autoware_planning_msgs::PathPoint>& path_points)
{
  std::vector<autoware_perception_msgs::DynamicObject> avoiding_objects;
  for(const auto& object: objects)
  {
    double dx = object.state.pose_covariance.pose.position.x - ego_pose.position.x;
    double dy = object.state.pose_covariance.pose.position.y - ego_pose.position.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if(object.state.twist_covariance.twist.linear.x< max_avoiding_objects_velocity_ms_)
    {
      avoiding_objects.push_back(object);
    }
  }
  
  double min_dist = 1999999;
  int min_ind = 0;
  for (int i = 0; i < path_points.size(); i++)
  {
    double dx1 = path_points[i].pose.position.x - ego_pose.position.x;
    double dy1 = path_points[i].pose.position.y - ego_pose.position.y;
    double dist = std::sqrt(dx1*dx1+dy1*dy1);
    if(dist < min_dist)
    {
      min_dist = dist;
      min_ind = i;
    }
  }
  int search_start_idx = 
    std::max((int)(min_ind - number_of_backward_detection_range_path_points_), 0);
  for (int i = search_start_idx; i < path_points.size(); i++)
  {
    for(const auto& avoiding_object: avoiding_objects)
    {
      double dist_from_path_point_to_obstalce;
      if(avoiding_object.semantic.type == avoiding_object.semantic.CAR ||
         avoiding_object.semantic.type == avoiding_object.semantic.BUS ||
         avoiding_object.semantic.type == avoiding_object.semantic.TRUCK)
      {
        double yaw = tf2::getYaw(avoiding_object.state.pose_covariance.pose.orientation);
        geometry_msgs::Point top_left;
        top_left.x = avoiding_object.shape.dimensions.x*0.5;
        top_left.y = avoiding_object.shape.dimensions.y*0.5;
        geometry_msgs::Point top_left_map;
        top_left_map.x =  std::cos(yaw)*top_left.x+  
                            -1*std::sin(yaw)*top_left.y;
        top_left_map.y =  std::sin(yaw)*top_left.x+  
                            std::cos(yaw)*top_left.y;
        top_left_map.x += avoiding_object.state.pose_covariance.pose.position.x;
        top_left_map.y += avoiding_object.state.pose_covariance.pose.position.y;
        
        geometry_msgs::Point top_right;
        top_right.x = avoiding_object.shape.dimensions.x*0.5;
        top_right.y = -1*avoiding_object.shape.dimensions.y*0.5;
        geometry_msgs::Point top_right_map;
        top_right_map.x =  std::cos(yaw)*top_right.x+  
                            -1*std::sin(yaw)*top_right.y;
        top_right_map.y =  std::sin(yaw)*top_right.x+  
                            std::cos(yaw)*top_right.y;
        top_right_map.x += avoiding_object.state.pose_covariance.pose.position.x;
        top_right_map.y += avoiding_object.state.pose_covariance.pose.position.y;
        
        geometry_msgs::Point bottom_left;
        bottom_left.x = -1*avoiding_object.shape.dimensions.x*0.5;
        bottom_left.y = avoiding_object.shape.dimensions.y*0.5;
        geometry_msgs::Point bottom_left_map;
        bottom_left_map.x =  std::cos(yaw)*bottom_left.x+  
                            -1*std::sin(yaw)*bottom_left.y;
        bottom_left_map.y =  std::sin(yaw)*bottom_left.x+  
                            std::cos(yaw)*bottom_left.y;
        bottom_left_map.x += avoiding_object.state.pose_covariance.pose.position.x;
        bottom_left_map.y += avoiding_object.state.pose_covariance.pose.position.y;
        
        geometry_msgs::Point bottom_right;
        bottom_right.x = -1*avoiding_object.shape.dimensions.x*0.5;
        bottom_right.y = -1*avoiding_object.shape.dimensions.y*0.5;
        geometry_msgs::Point bottom_right_map;
        bottom_right_map.x =  std::cos(yaw)*bottom_right.x+  
                            -1*std::sin(yaw)*bottom_right.y;
        bottom_right_map.y =  std::sin(yaw)*bottom_right.x+  
                            std::cos(yaw)*bottom_right.y;
        bottom_right_map.x += avoiding_object.state.pose_covariance.pose.position.x;
        bottom_right_map.y += avoiding_object.state.pose_covariance.pose.position.y;
        
        double dx1 = path_points[i].pose.position.x - top_left_map.x;
        double dy1 = path_points[i].pose.position.y - top_left_map.y;
        double dx2 = path_points[i].pose.position.x - top_right_map.x;
        double dy2 = path_points[i].pose.position.y - top_right_map.y;
        double dx3 = path_points[i].pose.position.x - bottom_right_map.x;
        double dy3 = path_points[i].pose.position.y - bottom_right_map.y;
        double dx4 = path_points[i].pose.position.x - bottom_left_map.x;
        double dy4 = path_points[i].pose.position.y - bottom_left_map.y;
        double dist1 = std::sqrt(dx1*dx1+dy1*dy1);
        double dist2 = std::sqrt(dx2*dx2+dy2*dy2);
        double dist3 = std::sqrt(dx3*dx3+dy3*dy3);
        double dist4 = std::sqrt(dx4*dx4+dy4*dy4);
        float min_dist = std::fmin(dist1, dist2);
        min_dist = std::fmin(min_dist, dist3);
        min_dist = std::fmin(min_dist, dist4);
        dist_from_path_point_to_obstalce = min_dist;
      }
      else
      {
        double dx1 = path_points[i].pose.position.x - 
                    avoiding_object.state.pose_covariance.pose.position.x;
        double dy1 = path_points[i].pose.position.y - 
                    avoiding_object.state.pose_covariance.pose.position.y;
        dist_from_path_point_to_obstalce = std::sqrt(dx1*dx1+dy1*dy1);
      }
      double dx2 = ego_pose.position.x - 
                  avoiding_object.state.pose_covariance.pose.position.x;
      double dy2 = ego_pose.position.y - 
                  avoiding_object.state.pose_covariance.pose.position.y;
      double dist_from_ego_to_obstacle = std::sqrt(dx2*dx2+dy2*dy2);
      
      if(dist_from_path_point_to_obstalce < detection_radius_around_path_point_ && 
         dist_from_ego_to_obstacle < detection_radius_from_ego_)
      {
        return true;
      }
    }
  }
  return false;
}

bool EBPathPlannerNode::detectAvoidingObjectsOnPoints(
  const std::vector<autoware_perception_msgs::DynamicObject>& objects,
  const std::vector<geometry_msgs::Point>& points)
{
  std::vector<autoware_perception_msgs::DynamicObject> avoiding_objects;
  for(const auto& object: objects)
  {
    if(object.state.twist_covariance.twist.linear.x< max_avoiding_objects_velocity_ms_)
    {
      avoiding_objects.push_back(object);
    }
  }
  
  for(const auto& point: points)
  {
    for(const auto& object: avoiding_objects)
    {
      double dx = point.x - object.state.pose_covariance.pose.position.x;
      double dy = point.y - object.state.pose_covariance.pose.position.y;
      double dist = std::sqrt(dx*dx+dy*dy);
      if(dist < exploring_minimum_radius_)
      {
        return true;
      }
    }
  }
  return false;
}


bool EBPathPlannerNode::alighWithPathPoints(
  const std::vector<autoware_planning_msgs::PathPoint>& path_points,
  std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  if(optimized_points.empty())
  {
    return false;
  }
  size_t previously_used_index = 0;
  for(size_t i = 0; i <  optimized_points.size(); i++)
  {
    // optimized_points[i].twist.linear.x = path_points.front().twist.linear.x;
    // optimized_points[i].pose.position.z = path_points.front().pose.position.z;
    bool flag = false;
    double min_dist = 9999999999;
    for (size_t j = 0; j < path_points.size(); j++)
    {
      double dx1 = optimized_points[i].pose.position.x - path_points[j].pose.position.x;
      double dy1 = optimized_points[i].pose.position.y - path_points[j].pose.position.y;
      double yaw = tf2::getYaw(path_points[j].pose.orientation);
      double dx2 = std::cos(yaw);
      double dy2 = std::sin(yaw);
      double inner_product = dx1*dx2+dy1*dy2;
      double dist = std::sqrt(dx1*dx1+dy1*dy1);
      if(inner_product < 0 && dist < min_dist)
      {
        min_dist = dist;
        optimized_points[i].pose.position.z = path_points[j].pose.position.z;
        optimized_points[i].twist.linear.x = path_points[j].twist.linear.x;
        previously_used_index = j;
        flag = true;
        break;
      }
    }
    if(!flag)
    {
      ROS_WARN_THROTTLE(1.0, "[EBPathPlanner] Could not find corresponding velocity in path points. Insert 0 velocity");
    }
  }
  // if(!path_points.empty())
  // {
  //   double dx = path_points.back().pose.position.x - optimized_points.back().pose.position.x;
  //   double dy = path_points.back().pose.position.y - optimized_points.back().pose.position.y;
  //   double dist = std::sqrt(dx*dx+dy*dy);
  //   if(path_points.back().twist.linear.x < 1e-6&&
  //      dist < 2)
  //   {
  //     optimized_points.back().twist.linear.x = 0;    
  //   }
    
  // }
}

bool EBPathPlannerNode::generateFineOptimizedPoints(
  const std::vector<autoware_planning_msgs::PathPoint>& path_points,
  const std::vector<autoware_planning_msgs::TrajectoryPoint>& merged_optimized_points,
  std::vector<autoware_planning_msgs::TrajectoryPoint>& fine_optimized_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  if(merged_optimized_points.empty())
  {
    return false;
  }
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  std::vector<double> tmp_z;
  std::vector<double> tmp_v;
  for(size_t i = 0; i <  merged_optimized_points.size(); i++)
  {
    tmp_x.push_back(merged_optimized_points[i].pose.position.x);
    tmp_y.push_back(merged_optimized_points[i].pose.position.y);
    tmp_z.push_back(merged_optimized_points[i].pose.position.z);
    tmp_v.push_back(merged_optimized_points[i].twist.linear.x);
  }
  
  if(!merged_optimized_points.empty()&&
     !path_points.empty())
  {
    double dx1 = merged_optimized_points.back().pose.position.x - 
                path_points.back().pose.position.x;
    double dy1 = merged_optimized_points.back().pose.position.y - 
                path_points.back().pose.position.y;
    double dist1 = std::sqrt(dx1*dx1+dy1*dy1);
    double yaw = tf2::getYaw(path_points.back().pose.orientation);
    double dx2 = std::cos(yaw);
    double dy2 = std::sin(yaw);
    double inner_product = dx1*dx2+dy1*dy2;
    if(dist1 < 5.0 && 
      inner_product < 0 &&
      path_points.back().twist.linear.x < 1e-6)
    {
      tmp_x.push_back(path_points.back().pose.position.x);
      tmp_y.push_back(path_points.back().pose.position.y);
      tmp_z.push_back(merged_optimized_points.back().pose.position.z);
      tmp_v.push_back(0);
    }  
  }
  std::vector<double> base_s = horibe_spline::calcEuclidDist(tmp_x, tmp_y);
  std::vector<double> new_s;
  for(double i = 0.0; 
      i <= base_s.back();
      i += 0.1)
  {
    new_s.push_back(i);
  }
  horibe_spline::SplineInterpolate spline;
  std::vector<double> new_x;
  std::vector<double> new_y;
  spline.interpolate(base_s, tmp_x, new_s, new_x);
  spline.interpolate(base_s, tmp_y, new_s, new_y);
  for(size_t i = 0; i < new_s.size(); i++)
  {
    autoware_planning_msgs::TrajectoryPoint traj_point;
    traj_point.pose.position.x = new_x[i]; 
    traj_point.pose.position.y = new_y[i]; 
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    if(i==new_s.size()-1)
    {
      double dx = new_x[i] - new_x[i-1]; 
      double dy = new_y[i] - new_y[i-1]; 
      yaw = std::atan2(dy, dx);
    }
    else
    {
      double dx = new_x[i+1] - new_x[i]; 
      double dy = new_y[i+1] - new_y[i]; 
      yaw = std::atan2(dy, dx);
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    traj_point.pose.orientation = tf2::toMsg(quaternion);
    auto it = std::lower_bound(base_s.begin(), 
                               base_s.end(),
                               new_s[i]);
    size_t ind = std::distance(base_s.begin(), it);
    traj_point.twist.linear.x = tmp_v[ind];
    traj_point.pose.position.z = tmp_z[ind];
    // traj_point.twist.linear.x = tmp_v.front();
    // traj_point.pose.position.z = tmp_z.front();
    fine_optimized_points.push_back(traj_point); 
  }
  if(tmp_v.back() < 1e-6 && 
     !fine_optimized_points.empty())
  {
    fine_optimized_points.back().twist.linear.x = 0;
  }
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "fine optimized points "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  return true;
}

void EBPathPlannerNode::getOccupancyGridValue(
  const nav_msgs::OccupancyGrid& og, 
  const int i, 
  const int j,
  unsigned char&value) 
{
  int i_flip = og.info.width - i-1;
  int j_flip = og.info.height - j-1;
  if(og.data[i_flip + j_flip*og.info.width] > 0)
  {
    value = 0;
  }
  else
  {
    value = 255;
  }
}

void EBPathPlannerNode::putOccupancyGridValue(
  nav_msgs::OccupancyGrid& og, 
  const int i, 
  const int j,
  const unsigned char&value) 
{
  int i_flip = og.info.width - i-1;
  int j_flip = og.info.height - j-1;
  og.data[i_flip + j_flip*og.info.width] = value;
}

bool EBPathPlannerNode::generateClearanceMap(
    const nav_msgs::OccupancyGrid& occupancy_grid,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const geometry_msgs::Pose& debug_ego_pose,
    cv::Mat& clearance_map)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  cv::Mat drivable_area = 
    cv::Mat(occupancy_grid.info.width, occupancy_grid.info.height, CV_8UC1);

  drivable_area.forEach<unsigned char>
  (
    [&](unsigned char &value, const int *position) -> void
    {
      getOccupancyGridValue(occupancy_grid, position[0], position[1], value);
    }
  );
  
  for(const auto& object: objects)
  {
    if(object.state.twist_covariance.twist.linear.x < max_avoiding_objects_velocity_ms_)
    {
      if(object.semantic.type == object.semantic.CAR)
      {
        double yaw = tf2::getYaw(object.state.pose_covariance.pose.orientation);
        geometry_msgs::Point top_left;
        top_left.x = object.shape.dimensions.x*0.5;
        top_left.y = object.shape.dimensions.y*0.5;
        geometry_msgs::Point top_left_map;
        top_left_map.x =  std::cos(yaw)*top_left.x+  
                            -1*std::sin(yaw)*top_left.y;
        top_left_map.y =  std::sin(yaw)*top_left.x+  
                            std::cos(yaw)*top_left.y;
        top_left_map.x += object.state.pose_covariance.pose.position.x;
        top_left_map.y += object.state.pose_covariance.pose.position.y;
        
        geometry_msgs::Point top_right;
        top_right.x = object.shape.dimensions.x*0.5;
        top_right.y = -1*object.shape.dimensions.y*0.5;
        geometry_msgs::Point top_right_map;
        top_right_map.x =  std::cos(yaw)*top_right.x+  
                            -1*std::sin(yaw)*top_right.y;
        top_right_map.y =  std::sin(yaw)*top_right.x+  
                            std::cos(yaw)*top_right.y;
        top_right_map.x += object.state.pose_covariance.pose.position.x;
        top_right_map.y += object.state.pose_covariance.pose.position.y;
        
        geometry_msgs::Point bottom_left;
        bottom_left.x = -1*object.shape.dimensions.x*0.5;
        bottom_left.y = object.shape.dimensions.y*0.5;
        geometry_msgs::Point bottom_left_map;
        bottom_left_map.x =  std::cos(yaw)*bottom_left.x+  
                            -1*std::sin(yaw)*bottom_left.y;
        bottom_left_map.y =  std::sin(yaw)*bottom_left.x+  
                            std::cos(yaw)*bottom_left.y;
        bottom_left_map.x += object.state.pose_covariance.pose.position.x;
        bottom_left_map.y += object.state.pose_covariance.pose.position.y;
        
        geometry_msgs::Point bottom_right;
        bottom_right.x = -1*object.shape.dimensions.x*0.5;
        bottom_right.y = -1*object.shape.dimensions.y*0.5;
        geometry_msgs::Point bottom_right_map;
        bottom_right_map.x =  std::cos(yaw)*bottom_right.x+  
                            -1*std::sin(yaw)*bottom_right.y;
        bottom_right_map.y =  std::sin(yaw)*bottom_right.x+  
                            std::cos(yaw)*bottom_right.y;
        bottom_right_map.x += object.state.pose_covariance.pose.position.x;
        bottom_right_map.y += object.state.pose_covariance.pose.position.y;
        
        geometry_msgs::Point top_left_map_in_image;
        geometry_msgs::Point top_right_map_in_image;
        geometry_msgs::Point bottom_left_map_in_image;
        geometry_msgs::Point bottom_right_map_in_image;
        tmp1::transformMapToImage(
              top_left_map, 
              occupancy_grid.info,
              top_left_map_in_image);
        tmp1::transformMapToImage(
              top_right_map, 
              occupancy_grid.info,
              top_right_map_in_image);
        tmp1::transformMapToImage(
              bottom_left_map, 
              occupancy_grid.info,
              bottom_left_map_in_image);
        tmp1::transformMapToImage(
              bottom_right_map, 
              occupancy_grid.info,
              bottom_right_map_in_image);
        cv::Point top_left_image_point = 
          cv::Point(top_left_map_in_image.x, top_left_map_in_image.y);
        cv::Point top_right_image_point = 
          cv::Point(top_right_map_in_image.x, top_right_map_in_image.y);
        cv::Point bottom_right_image_point = 
          cv::Point(bottom_right_map_in_image.x, bottom_right_map_in_image.y);
        cv::Point bottom_left_image_point = 
          cv::Point(bottom_left_map_in_image.x, bottom_left_map_in_image.y);
        cv::clipLine(
          cv::Rect(0, 0, drivable_area.size().width, drivable_area.size().height),
          top_left_image_point,
          top_right_image_point);
        cv::clipLine(
          cv::Rect(0, 0, drivable_area.size().width, drivable_area.size().height),
          top_right_image_point,
          bottom_right_image_point);
        cv::clipLine(
          cv::Rect(0, 0, drivable_area.size().width, drivable_area.size().height),
          bottom_right_image_point,
          bottom_left_image_point);
        cv::clipLine(
          cv::Rect(0, 0, drivable_area.size().width, drivable_area.size().height),
          bottom_left_image_point,
          top_left_image_point);
          
        cv::line(drivable_area,
                  top_left_image_point,
                  top_right_image_point,
                  cv::Scalar(0), 10);
        cv::line(drivable_area,
                  top_right_image_point,
                  bottom_right_image_point,
                  cv::Scalar(0), 10);
        cv::line(drivable_area,
                  bottom_right_image_point,
                  bottom_left_image_point,
                  cv::Scalar(0), 10);
        cv::line(drivable_area,
                  bottom_left_image_point,
                  top_left_image_point,
                  cv::Scalar(0), 10);
      }
      else
      {
        geometry_msgs::Point point_in_image;
        if(tmp1::transformMapToImage(
                object.state.pose_covariance.pose.position, 
                occupancy_grid.info,
                point_in_image))
        {
          cv::circle(drivable_area, 
                    cv::Point(point_in_image.x, point_in_image.y), 
                    15, 
                    cv::Scalar(0),
                    -1);
            
        } 
        
      }      
    }
  }
  
  cv::distanceTransform(drivable_area, clearance_map, cv::DIST_L2, 5);
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << " conversion & edt "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  if(is_debug_clearance_map_mode_)
  {
    cv::Mat tmp;  
    clearance_map.copyTo(tmp);
    cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", tmp);
    cv::waitKey(10);
  }
  if(is_debug_drivable_area_mode_)
  {
    cv::Mat tmp;
    drivable_area.copyTo(tmp);
    cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
    cv::imshow("image", tmp);
    cv::waitKey(10);
  }
  
  if(is_publishing_clearance_map_as_occupancy_grid_)
  {
    cv::Mat tmp;
    clearance_map.copyTo(tmp);
    cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    nav_msgs::OccupancyGrid clearance_map_in_og = occupancy_grid;
    tmp.forEach<unsigned char>
    (
      [&](const unsigned char &value, const int *position) -> void
      {
        putOccupancyGridValue(clearance_map_in_og, position[0], position[1], value);
      }
    );
    debug_clearance_map_in_occupancy_grid_pub_.publish(clearance_map_in_og);
  }
}

bool EBPathPlannerNode::convertPathToSmoothTrajectory(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& smoothed_points,
    std::vector<geometry_msgs::Point>& debug_constrain_points,
    std::vector<geometry_msgs::Point>& debug_interpolated_points_used_for_optimization)
{
  std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
  eb_path_smoother_ptr_->generateOptimizedPath(
    ego_pose,
    path_points,
    optimized_points,
    debug_constrain_points,
    debug_interpolated_points_used_for_optimization);
  
  if(optimized_points.empty())
  {
    ROS_WARN("Path size %d is not enough for smoothing; Relay path to trajetory ",
      (int)path_points.size());
    for(const auto& point: path_points)
    {
      autoware_planning_msgs::TrajectoryPoint smoothed_point;
      smoothed_point.pose = point.pose;
      smoothed_point.twist = point.twist;
      smoothed_points.push_back(smoothed_point);
    }
    return false;
  }
  autoware_planning_msgs::Trajectory tmp_traj;
  tmp_traj.points = optimized_points;
  debug_traj_pub_.publish(tmp_traj);
  
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  std::vector<double> tmp_z;
  std::vector<double> tmp_v;
  for(size_t i = 0; i <  optimized_points.size(); i++)
  {
    //backward check
    if(i > 1)
    {
      double dx1 = optimized_points[i].pose.position.x - 
                  optimized_points[i-1].pose.position.x;
      double dy1 = optimized_points[i].pose.position.y - 
                  optimized_points[i-1].pose.position.y;
      double dx2 = optimized_points[i-1].pose.position.x - 
                  optimized_points[i-2].pose.position.x;
      double dy2 = optimized_points[i-1].pose.position.y - 
                  optimized_points[i-2].pose.position.y;
      double inner_product = dx1*dx2 + dy1*dy2;
      if(inner_product < 0)
      {
        ROS_INFO("Path points might go backward");
      }
    }
    tmp_x.push_back(optimized_points[i].pose.position.x);
    tmp_y.push_back(optimized_points[i].pose.position.y);
    tmp_z.push_back(optimized_points[i].pose.position.z);
    tmp_v.push_back(optimized_points[i].twist.linear.x);
  }
  
  std::vector<double> base_s = horibe_spline::calcEuclidDist(tmp_x, tmp_y);
  std::vector<double> new_s;
  for(double i = 0.0; 
      i <= base_s.back();
      i += delta_arc_length_for_path_smoothing_)
  {
    new_s.push_back(i);
  }
  
  horibe_spline::SplineInterpolate spline;
  std::vector<double> new_x;
  std::vector<double> new_y;
  spline.interpolate(base_s, tmp_x, new_s, new_x);
  spline.interpolate(base_s, tmp_y, new_s, new_y);
  for(size_t i = 0; i < new_s.size(); i++)
  {
    autoware_planning_msgs::TrajectoryPoint traj_point;
    traj_point.pose.position.x = new_x[i]; 
    traj_point.pose.position.y = new_y[i]; 
    if(std::isnan(new_x[i]) || std::isnan(new_y[i]))
    {
      ROS_WARN("[path2tracjectory]: Interpolation gets nan value. Relay path to trajectory, but point interval is more than 0.1m");
      return false;
    }
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    if(i==new_s.size()-1)
    {
      double dx = new_x[i] - new_x[i-1]; 
      double dy = new_y[i] - new_y[i-1]; 
      yaw = std::atan2(dy, dx);
    }
    else
    {
      double dx = new_x[i+1] - new_x[i]; 
      double dy = new_y[i+1] - new_y[i]; 
      yaw = std::atan2(dy, dx);
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    traj_point.pose.orientation = tf2::toMsg(quaternion);
    auto it = std::lower_bound(base_s.begin(), 
                              base_s.end(),
                              new_s[i]);
    size_t ind = std::distance(base_s.begin(), it);
    traj_point.twist.linear.x = tmp_v[ind];
    traj_point.pose.position.z = tmp_z[ind];
    smoothed_points.push_back(traj_point); 
  }
  //sanity check for last point
  if(tmp_v.back()<1e-8 &&
    smoothed_points.back().twist.linear.x > 1e-8)
  {
    smoothed_points.back().twist.linear.x = 0;
  }
  return true;
}
} // namespace path_planner
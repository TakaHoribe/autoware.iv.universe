
#include <std_msgs/Bool.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_perception_msgs/DynamicObject.h>
// #include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>

// #include <lanelet2_extension/utility/message_conversion.h>


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
    
  
  
  is_relay_path_sub_ = private_nh_.subscribe("/is_relay_path", 1, 
                           &EBPathPlannerNode::isRelayPathCallback, this);
  twist_sub_ = private_nh_.subscribe("/current_velocity", 1, 
                           &EBPathPlannerNode::currentVelocityCallback, this);
  objects_sub_ = private_nh_.subscribe("/perception/prediction/objects", 10,
                           &EBPathPlannerNode::objectsCallback, this);
  private_nh_.param<bool>("is_relay_path_mode", 
                           is_relaying_path_mode_,false);
  private_nh_.param<bool>("use_optimization_when_relaying", 
                           use_optimization_when_relaying_,true);
  private_nh_.param<int>("number_of_fixing_explored_points", 
                          number_of_fixing_explored_points_, 10);
  private_nh_.param<double>("backward_fixing_distance", 
                             backward_fixing_distance_, 5);
  private_nh_.param<double>("exploring_minumum_radius", 
                             exploring_minimum_radius_, 1.3);
  private_nh_.param<double>("forward_fixing_distance", 
                             forward_fixing_distance_, 10.0);
  private_nh_.param<double>("delta_arc_length_for_path_smoothing", 
                             delta_arc_length_for_path_smoothing_, 1.0);
  private_nh_.param<double>("delta_arc_length_for_explored_points", 
                             delta_arc_length_for_explored_points_, 0.3);
  private_nh_.param<double>("max_avoiding_objects_velocity_ms", 
                             max_avoiding_objects_velocity_ms_, 0.1);
  doResetting();
}

EBPathPlannerNode::~EBPathPlannerNode() {}

void EBPathPlannerNode::callback(const autoware_planning_msgs::Path &input_path_msg,
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
  bool is_objects_detected = 
    detectAvoidingObjectsOnPath(
      self_pose, 
      in_objects_ptr_->objects,
      input_path_msg.points);
  if(!is_objects_detected)
  {
    generateSmoothTrajectory(
      self_pose,
      input_path_msg,
      output_trajectory_msg);
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
                 clearance_map,
                 input_path_msg.drivable_area.info,
                 previous_explored_points_ptr_))
    {
      doResetting();
    }
    
    geometry_msgs::Pose start_exploring_pose;
    std::vector<geometry_msgs::Point> fixed_explored_points; 
    generateFixedExploredPoints(
        self_pose, 
        previous_explored_points_ptr_,
        clearance_map,
        input_path_msg.drivable_area.info, 
        fixed_explored_points);
    if(fixed_explored_points.empty())
    {
      ROS_INFO("[EBPathPlanner] No fixing explored points");
      double min_dist = 99999999;
      int min_ind = -1;
      if(!input_path_msg.points.empty())
      {
        for (int i = 0; i < input_path_msg.points.size(); i++)
        {
          double dx1 = input_path_msg.points[i].pose.position.x - self_pose.position.x;
          double dy1 = input_path_msg.points[i].pose.position.y - self_pose.position.y;
          double dist = std::sqrt(dx1*dx1+dy1*dy1);
          if(dist < min_dist)
          {
            min_dist = dist;
            min_ind = i;
          }
        }
        int back_ind = std::max(min_ind - 3, 0);
        if(min_ind != -1)
        {
          start_exploring_pose = input_path_msg.points[back_ind].pose;
        }
      }
      else
      {
        ROS_WARN("Path is empty");
        return;
      }
    }
    else
    {
      start_exploring_pose.position = fixed_explored_points.back();
    }
    
    std::vector<geometry_msgs::Point> explored_points;
    geometry_msgs::Point debug_goal_point;
    std::vector<geometry_msgs::Point> debug_rearranged_points;
    bool is_explore_success = 
      modify_reference_path_ptr_->generateModifiedPath(
            self_pose, 
            start_exploring_pose,
            input_path_msg.points,
            in_objects_ptr_->objects,
            explored_points,
            clearance_map, 
            input_path_msg.drivable_area.info,
            debug_goal_point,
            debug_rearranged_points);
    if(!is_explore_success)
    {
      generateSmoothTrajectory(
        self_pose,
        input_path_msg,
        output_trajectory_msg);
      // doResetting();
      return;
    }
    for (int i = 1; i < explored_points.size(); i++)
    {
      fixed_explored_points.push_back(explored_points[i]);
    }
    explored_points = fixed_explored_points;
    previous_explored_points_ptr_ = 
      std::make_unique<std::vector<geometry_msgs::Point>>(explored_points);    
    
    std::vector<geometry_msgs::Point> debug_interpolated_points;
    std::vector<geometry_msgs::Point> debug_cached_explored_points;
    std::vector<geometry_msgs::Point> debug_boundary_points;
    std::vector<geometry_msgs::Point> debug_boundary_lb_points;
    std::vector<geometry_msgs::Point> debug_boundary_ub_points;
    std::vector<geometry_msgs::Point> debug_fixed_optimization_points;
    std::vector<geometry_msgs::Point> debug_variable_optimization_points;
    std::vector<geometry_msgs::Point> debug_constrain_points;
    std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
    eb_path_smoother_ptr_->generateOptimizedExploredPoints(
          input_path_msg.points,
          explored_points, 
          start_exploring_pose,
          self_pose,
          clearance_map, 
          input_path_msg.drivable_area.info,
          debug_interpolated_points,
          debug_cached_explored_points,
          debug_boundary_points,
          debug_boundary_lb_points,
          debug_boundary_ub_points,
          debug_fixed_optimization_points,
          debug_variable_optimization_points,
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
    visualization_msgs::Marker debug_cubic_spline;
    debug_cubic_spline.lifetime = ros::Duration(1.0);
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
    for(const auto& point: explored_points)
    {
      debug_cubic_spline.points.push_back(point);
    }
    if(!debug_cubic_spline.points.empty())
    {
      marker_array.markers.push_back(debug_cubic_spline);
    }
    unique_id++;
    
    // visualize cubic spline point
    visualization_msgs::Marker debug_goal_point_marker;
    debug_goal_point_marker.lifetime = ros::Duration(1.0);
    debug_goal_point_marker.header = input_path_msg.header;
    debug_goal_point_marker.ns = std::string("goal_point_marker");
    debug_goal_point_marker.action = visualization_msgs::Marker::MODIFY;
    debug_goal_point_marker.pose.orientation.w = 1.0;
    debug_goal_point_marker.id = unique_id;
    debug_goal_point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_goal_point_marker.scale.x = 1.0f;
    debug_goal_point_marker.scale.y = 0.1f;
    debug_goal_point_marker.scale.z = 0.1f;
    debug_goal_point_marker.color.r = 1.0f;
    debug_goal_point_marker.color.a = 0.999;
    debug_goal_point_marker.points.push_back(debug_goal_point);
    marker_array.markers.push_back(debug_goal_point_marker);
    unique_id++;
    
    visualization_msgs::Marker debug_start_point_marker;
    debug_start_point_marker.lifetime = ros::Duration(1.0);
    debug_start_point_marker.header = input_path_msg.header;
    debug_start_point_marker.ns = std::string("start_point_marker");
    debug_start_point_marker.action = visualization_msgs::Marker::MODIFY;
    debug_start_point_marker.pose.orientation.w = 1.0;
    debug_start_point_marker.id = unique_id;
    debug_start_point_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_start_point_marker.scale.x = 1.0f;
    debug_start_point_marker.scale.y = 0.1f;
    debug_start_point_marker.scale.z = 0.1f;
    debug_start_point_marker.color.r = 1.0f;
    debug_start_point_marker.color.a = 0.999;
    debug_start_point_marker.points.push_back(start_exploring_pose.position);
    marker_array.markers.push_back(debug_start_point_marker);
    unique_id++;
    
    // visualize cubic spline point
    visualization_msgs::Marker debug_interpolated_points_marker;
    debug_interpolated_points_marker.lifetime = ros::Duration(1.0);
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
    visualization_msgs::Marker debug_optimized_points_marker;
    debug_optimized_points_marker.lifetime = ros::Duration(1.0);
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
    
    visualization_msgs::Marker boundary_points_marker;
    boundary_points_marker.lifetime = ros::Duration(1.0);
    boundary_points_marker.header = input_path_msg.header;
    boundary_points_marker.ns = std::string("boundary_points_marker");
    boundary_points_marker.action = visualization_msgs::Marker::MODIFY;
    boundary_points_marker.pose.orientation.w = 1.0;
    boundary_points_marker.id = unique_id;
    boundary_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    boundary_points_marker.scale.x = 0.5f;
    boundary_points_marker.scale.y = 0.1f;
    boundary_points_marker.scale.z = 0.1f;
    boundary_points_marker.color.r = 1.0f;
    // boundary_points_marker.color.g = 1.0f;
    boundary_points_marker.color.a = 0.999;
    for (int i = 0; i < debug_boundary_points.size(); i++)
    {
      boundary_points_marker.points.push_back(debug_boundary_points[i]);
    }
    if(!boundary_points_marker.points.empty())
    {
      marker_array.markers.push_back(boundary_points_marker);
    }
    
    unique_id++;
    visualization_msgs::Marker lb_boundary_points_marker;
    lb_boundary_points_marker.lifetime = ros::Duration(1.0);
    lb_boundary_points_marker.header = input_path_msg.header;
    lb_boundary_points_marker.ns = std::string("lb_boundary_points_marker");
    lb_boundary_points_marker.action = visualization_msgs::Marker::MODIFY;
    lb_boundary_points_marker.pose.orientation.w = 1.0;
    lb_boundary_points_marker.id = unique_id;
    lb_boundary_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    lb_boundary_points_marker.scale.x = 0.5f;
    lb_boundary_points_marker.scale.y = 0.1f;
    lb_boundary_points_marker.scale.z = 0.1f;
    lb_boundary_points_marker.color.r = 1.0f;
    lb_boundary_points_marker.color.b = 1.0f;
    // lb_boundary_points_marker.color.g = 1.0f;
    lb_boundary_points_marker.color.a = 0.999;
    for (int i = 0; i < debug_boundary_lb_points.size(); i++)
    {
      lb_boundary_points_marker.points.push_back(debug_boundary_lb_points[i]);
    }
    if(!lb_boundary_points_marker.points.empty())
    {
      marker_array.markers.push_back(lb_boundary_points_marker);
    }
    
    unique_id++;
    visualization_msgs::Marker ub_boundary_points_marker;
    ub_boundary_points_marker.lifetime = ros::Duration(1.0);
    ub_boundary_points_marker.header = input_path_msg.header;
    ub_boundary_points_marker.ns = std::string("ub_boundary_points_marker");
    ub_boundary_points_marker.action = visualization_msgs::Marker::MODIFY;
    ub_boundary_points_marker.pose.orientation.w = 1.0;
    ub_boundary_points_marker.id = unique_id;
    ub_boundary_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    ub_boundary_points_marker.scale.x = 0.5f;
    ub_boundary_points_marker.scale.y = 0.1f;
    ub_boundary_points_marker.scale.z = 0.1f;
    ub_boundary_points_marker.color.r = 1.0f;
    ub_boundary_points_marker.color.g = 1.0f;
    // ub_boundary_points_marker.color.g = 1.0f;
    ub_boundary_points_marker.color.a = 0.999;
    for (int i = 0; i < debug_boundary_ub_points.size(); i++)
    {
      ub_boundary_points_marker.points.push_back(debug_boundary_ub_points[i]);
    }
    if(!ub_boundary_points_marker.points.empty())
    {
      marker_array.markers.push_back(ub_boundary_points_marker);
    }
    
    visualization_msgs::Marker constrain_points_marker;
    constrain_points_marker.lifetime = ros::Duration(1.0);
    constrain_points_marker.header = input_path_msg.header;
    constrain_points_marker.ns = std::string("constrain_points_marker");
    constrain_points_marker.action = visualization_msgs::Marker::MODIFY;
    constrain_points_marker.pose.orientation.w = 1.0;
    constrain_points_marker.id = unique_id;
    constrain_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    constrain_points_marker.scale.x = 0.5f;
    constrain_points_marker.scale.y = 0.1f;
    constrain_points_marker.scale.z = 0.1f;
    constrain_points_marker.color.r = 1.0f;
    constrain_points_marker.color.g = 1.0f;
    // constrain_points_marker.color.g = 1.0f;
    constrain_points_marker.color.a = 0.999;
    for (int i = 0; i < debug_constrain_points.size(); i++)
    {
      constrain_points_marker.points.push_back(debug_constrain_points[i]);
    }
    if(!constrain_points_marker.points.empty())
    {
      marker_array.markers.push_back(constrain_points_marker);
    }
    unique_id++;
    
    visualization_msgs::Marker start_arrow_marker;
    start_arrow_marker.lifetime = ros::Duration(1.0);
    start_arrow_marker.header = input_path_msg.header;
    start_arrow_marker.ns = std::string("start_arrow_marker");
    start_arrow_marker.action = visualization_msgs::Marker::MODIFY;
    start_arrow_marker.pose.orientation.w = 1.0;
    start_arrow_marker.id = unique_id;
    start_arrow_marker.type = visualization_msgs::Marker::ARROW;
    start_arrow_marker.scale.x = 0.8f;
    start_arrow_marker.scale.y = 0.3f;
    start_arrow_marker.scale.z = 0.3f;
    start_arrow_marker.color.r = 1.0f;
    start_arrow_marker.color.a = 0.999;
    start_arrow_marker.pose = start_exploring_pose;
    marker_array.markers.push_back(start_arrow_marker);
    unique_id++;
    
    for (size_t i = 0; i < explored_points.size(); i++)
    {
      visualization_msgs::Marker text_marker;
      text_marker.lifetime = ros::Duration(1.0);
      text_marker.header = input_path_msg.header;
      text_marker.ns = std::string("text_explored_points");
      text_marker.action = visualization_msgs::Marker::MODIFY;
      text_marker.pose.orientation.w = 1.0;
      text_marker.id = unique_id;
      text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      text_marker.scale.z = 1.0f;
      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.a = 0.999;
      text_marker.text = std::to_string((int)i);
      text_marker.pose.position = explored_points[i];
      text_marker.pose.orientation.w = 1.0;
      marker_array.markers.push_back(text_marker);
      unique_id++;
    }
    
    for (size_t i = 0; i < optimized_points.size(); i++)
    {
      visualization_msgs::Marker text_marker;
      text_marker.lifetime = ros::Duration(1.0);
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
}

void EBPathPlannerNode::doResetting()
{
  modify_reference_path_ptr_ = std::make_unique<ModifyReferencePath>(
                  exploring_minimum_radius_,
                  backward_fixing_distance_);
  eb_path_smoother_ptr_ = std::make_unique<EBPathSmoother>(
              exploring_minimum_radius_,
              backward_fixing_distance_,
              forward_fixing_distance_,
              delta_arc_length_for_path_smoothing_,
              delta_arc_length_for_explored_points_);
  previous_explored_points_ptr_ = 
    std::make_unique<std::vector<geometry_msgs::Point>>();
}

void EBPathPlannerNode::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  in_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(msg);
}

void EBPathPlannerNode::isRelayPathCallback(const std_msgs::Bool& msg)
{
  is_relaying_path_mode_ = msg.data;
}

void EBPathPlannerNode::objectsCallback(
  const autoware_perception_msgs::DynamicObjectArray& msg)
{
  in_objects_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(msg);
}

bool EBPathPlannerNode::needReset(
  const geometry_msgs::Point& current_ego_point,
  const cv::Mat& clearance_map,
  const nav_msgs::MapMetaData& map_info,
  const std::unique_ptr<std::vector<geometry_msgs::Point>>& previous_explored_points_ptr)
{
  bool is_need_reset = false;
  //check1
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
    ROS_WARN(
    "[EBPathPlanner] Reset eb path planner since current ego vehicle is outside of drivavle area");
    is_need_reset = true;
    return is_need_reset;
  }
  
  //check3
  int count = 0;
  if(previous_explored_points_ptr_)
  {
    for(const auto& point: *previous_explored_points_ptr_)
    {
      geometry_msgs::Point point_in_image;
      if(tmp1::transformMapToImage(
              point, 
              map_info,
              point_in_image))
      {
        float clearance = 
            clearance_map.ptr<float>((int)point_in_image.y)
                                    [(int)point_in_image.x]*map_info.resolution;
        if(clearance < 1e-6)
        {
          // ROS_WARN("count %d", count);
          ROS_WARN(
            "[EBPathPlanner] Reset eb path planner since explored points are outside of drivavle area");
          is_need_reset = true;
          return is_need_reset;
        }
      }
      count++;
    }
  }
  //check4
  if(is_relaying_path_mode_)
  {
    ROS_WARN(
    "[EBPathPlanner] Reset eb path planner since is_relay_path_mode is enabled");
    is_need_reset = true;
  }
  return is_need_reset;
}

bool EBPathPlannerNode::generateSmoothTrajectory(
  const geometry_msgs::Pose& ego_pose,
  const autoware_planning_msgs::Path& input_path,
  autoware_planning_msgs::Trajectory& output_trajectory)
{
  std::vector<geometry_msgs::Point> debug_fixed_optimzied_points_used_for_constrain;
  std::vector<geometry_msgs::Point> debug_interpolated_points_used_for_optimization;
  std::vector<autoware_planning_msgs::TrajectoryPoint> smooth_trajectory_points;
  convertPathToSmoothTrajectory(
    ego_pose,
    input_path.points,
    smooth_trajectory_points,
    debug_fixed_optimzied_points_used_for_constrain,
    debug_interpolated_points_used_for_optimization);
  std::cout << "smooth trajectory points "<< smooth_trajectory_points.size() << std::endl;
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
  visualization_msgs::Marker smooth_debug_fixed_constrain_marker;
  smooth_debug_fixed_constrain_marker.lifetime = ros::Duration(1.0);
  smooth_debug_fixed_constrain_marker.header = input_path.header;
  smooth_debug_fixed_constrain_marker.ns = std::string("smooth_debug_fixed_constrain_marker");
  smooth_debug_fixed_constrain_marker.action = visualization_msgs::Marker::MODIFY;
  smooth_debug_fixed_constrain_marker.pose.orientation.w = 1.0;
  smooth_debug_fixed_constrain_marker.id = unique_id;
  smooth_debug_fixed_constrain_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  smooth_debug_fixed_constrain_marker.scale.x = 0.5f;
  smooth_debug_fixed_constrain_marker.scale.y = 0.1f;
  smooth_debug_fixed_constrain_marker.scale.z = 0.1f;
  smooth_debug_fixed_constrain_marker.color.r = 1.0f;
  smooth_debug_fixed_constrain_marker.color.g = 1.0f;
  smooth_debug_fixed_constrain_marker.color.a = 0.999;
  for (int i = 0; i < debug_fixed_optimzied_points_used_for_constrain.size(); i++)
  {
    smooth_debug_fixed_constrain_marker.points.push_back(debug_fixed_optimzied_points_used_for_constrain[i]);
  }
  if(!smooth_debug_fixed_constrain_marker.points.empty())
  {
    marker_array.markers.push_back(smooth_debug_fixed_constrain_marker);
  }
  unique_id++;
  
  visualization_msgs::Marker smooth_interpolated_points;
  smooth_interpolated_points.lifetime = ros::Duration(1.0);
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
  // smooth_interpolated_points.color.g = 1.0f;
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
    double dx = path_points[i].pose.position.x - ego_pose.position.x;
    double dy = path_points[i].pose.position.y - ego_pose.position.y;
    double dist = std::sqrt(dx*dx+dy*dy);
    if(dist < min_dist)
    {
      min_dist = dist;
      min_ind = i;
    }
  }
  for (int i = min_ind; i < path_points.size(); i++)
  {
    for(const auto& avoiding_object: avoiding_objects)
    {
      double dx = path_points[i].pose.position.x - 
                  avoiding_object.state.pose_covariance.pose.position.x;
      double dy = path_points[i].pose.position.y - 
                  avoiding_object.state.pose_covariance.pose.position.y;
      double dist = std::sqrt(dx*dx+dy*dy);
      if(dist < exploring_minimum_radius_*2)
      {
        return true;
      }
    }
  }
  return false;
}

bool EBPathPlannerNode::generateFixedExploredPoints(
  const geometry_msgs::Pose& ego_pose,
  const std::unique_ptr<std::vector<geometry_msgs::Point>>& previous_explored_points_ptr,
  const cv::Mat& clearance_map,
  const nav_msgs::MapMetaData& map_info,
  std::vector<geometry_msgs::Point>& fixed_explored_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  if(!previous_explored_points_ptr)
  {
    ROS_WARN("[EBPathPlanner] previous_explored_points_ptr is nullptr; no fixed points are passed");
    return false;
  }
  double min_dist = 1000000;
  int min_ind = 0;
  for (int i = 0; i < previous_explored_points_ptr->size(); i++)
  {
    double dx = previous_explored_points_ptr->at(i).x -  ego_pose.position.x; 
    double dy = previous_explored_points_ptr->at(i).y -  ego_pose.position.y; 
    double dist = std::sqrt(dx*dx+dy*dy);
    // std::cout << "dist "<< dist << std::endl;
    if(dist < min_dist)
    {
      min_dist = dist;
      min_ind = i;
    }
  }
  // std::cout << "prv opt size "<< previous_explored_points_ptr->size() << std::endl;
  std::cout << "min dist "<< min_dist << std::endl;
  const double keep_distance = forward_fixing_distance_ + backward_fixing_distance_;
  int forward_fixing_idx = 
    std::min((int)(min_ind+forward_fixing_distance_/exploring_minimum_radius_),
             (int)previous_explored_points_ptr->size());
  const double backward_keep_distance = 
    keep_distance - (forward_fixing_idx-min_ind)*exploring_minimum_radius_;
  int backward_fixing_idx = 
    std::max((int)(min_ind-backward_keep_distance/exploring_minimum_radius_),
                  0);
  int origin_valid_prev_explored_points_ind = 0;
  for (int i = 0; i < forward_fixing_idx; i++)
  {
    geometry_msgs::Point point_in_image;
    if(tmp1::transformMapToImage(
            previous_explored_points_ptr_->at(i), 
            map_info,
            point_in_image))
    {
       float clearance = 
          clearance_map.ptr<float>((int)point_in_image.y)
                                  [(int)point_in_image.x]*map_info.resolution;
       origin_valid_prev_explored_points_ind = i;
       if(clearance > 0)
       {
         break;
       }
       else
       {
         ROS_WARN_THROTTLE(1.0, 
              "[EBPathPlanner] Discard fixed explored points since they are out of dirvable area");
       }
    }
  }
  int valid_backward_fixing_idx =  std::max(origin_valid_prev_explored_points_ind, backward_fixing_idx);
  valid_backward_fixing_idx = std::min(valid_backward_fixing_idx, forward_fixing_idx);
  std::cout << "backward fixing idx "<< backward_fixing_idx << std::endl;       
  std::cout << "valid backward fixing idx "<< valid_backward_fixing_idx << std::endl;       
  std::cout << "backward keep distance  "<< backward_keep_distance << std::endl;       
  for (int i = valid_backward_fixing_idx; i < forward_fixing_idx; i++)
  {
    fixed_explored_points.push_back(previous_explored_points_ptr->at(i));
  }     
      
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "generate fixed points "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  return true;
}

bool EBPathPlannerNode::alighWithPathPoints(
  const std::vector<autoware_planning_msgs::PathPoint>& path_points,
  std::vector<autoware_planning_msgs::TrajectoryPoint>& merged_optimized_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  if(merged_optimized_points.empty())
  {
    return false;
  }
  size_t previously_used_index = 0;
  for(size_t i = 0; i <  merged_optimized_points.size(); i++)
  {
    bool flag = false;
    for (size_t j = previously_used_index; j < path_points.size(); j++)
    {
      double dx1 = merged_optimized_points[i].pose.position.x - path_points[j].pose.position.x;
      double dy1 = merged_optimized_points[i].pose.position.y - path_points[j].pose.position.y;
      double yaw = tf2::getYaw(path_points[j].pose.orientation);
      double dx2 = std::cos(yaw);
      double dy2 = std::sin(yaw);
      double inner_product = dx1*dx2+dy1*dy2;
      if(inner_product < 0)
      {
        merged_optimized_points[i].pose.position.z = path_points[j].pose.position.z;
        merged_optimized_points[i].twist.linear.x = path_points[j].twist.linear.x;
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
  double dx = merged_optimized_points.back().pose.position.x - 
                path_points.back().pose.position.x;
  double dy = merged_optimized_points.back().pose.position.y -
                path_points.back().pose.position.y;
  double dist = std::sqrt(dx*dx+dy*dy);
  double diff_dist_for_goal_thres = 10;
  if(dist < diff_dist_for_goal_thres && !merged_optimized_points.empty())
  {
    merged_optimized_points.back().twist.linear.x = 0;
  }
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
    if(dist1 < 1.0 && 
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
    fine_optimized_points.push_back(traj_point); 
  }
  double dx = fine_optimized_points.back().pose.position.x - 
                path_points.back().pose.position.x;
  double dy = fine_optimized_points.back().pose.position.y -
                path_points.back().pose.position.y;
  double dist = std::sqrt(dx*dx+dy*dy);
  double diff_dist_for_goal_thres = 10;
  if(dist < diff_dist_for_goal_thres && !fine_optimized_points.empty())
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

void EBPathPlannerNode::getOccupancyGridValue(const nav_msgs::OccupancyGrid& og, 
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
  
  cv::distanceTransform(drivable_area, clearance_map, cv::DIST_L2, 5);
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << " conversion & edt "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
}

bool EBPathPlannerNode::convertPathToSmoothTrajectory(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& smoothed_points,
    std::vector<geometry_msgs::Point>& debug_fixed_optimzied_points_used_for_constrain,
    std::vector<geometry_msgs::Point>& debug_interpolated_points_used_for_optimization)
{
  if(use_optimization_when_relaying_)
  {
    
    std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
    eb_path_smoother_ptr_->generateOptimizedPath(
      ego_pose,
      path_points,
      optimized_points,
      debug_fixed_optimzied_points_used_for_constrain,
      debug_interpolated_points_used_for_optimization);
      
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
  }
  else
  {
    
    std::vector<double> tmp_x;
    std::vector<double> tmp_y;
    std::vector<double> tmp_z;
    std::vector<double> tmp_v;
    for(size_t i = 0; i <  path_points.size(); i++)
    {
      //backward check
      if(i > 1)
      {
        double dx1 = path_points[i].pose.position.x - 
                    path_points[i-1].pose.position.x;
        double dy1 = path_points[i].pose.position.y - 
                    path_points[i-1].pose.position.y;
        double dx2 = path_points[i-1].pose.position.x - 
                    path_points[i-2].pose.position.x;
        double dy2 = path_points[i-1].pose.position.y - 
                    path_points[i-2].pose.position.y;
        double inner_product = dx1*dx2 + dy1*dy2;
        if(inner_product < 0)
        {
          ROS_INFO("Path points might go backward");
        }
      }
      
      //resolution check
      if(i > 0)
      {
        double dx = path_points[i].pose.position.x - 
                    path_points[i-1].pose.position.x;
        double dy = path_points[i].pose.position.y - 
                    path_points[i-1].pose.position.y;
        double dist = std::sqrt(dx*dx + dy*dy); 
        if(dist < 0.1)
        {
          if(path_points[i].twist.linear.x < 
            path_points[i-1].twist.linear.x)
          {
            //replace point with lower velocity
            tmp_x.pop_back();
            tmp_y.pop_back();
            tmp_z.pop_back();
            tmp_v.pop_back();
            tmp_x.push_back(path_points[i].pose.position.x);
            tmp_y.push_back(path_points[i].pose.position.y);
            tmp_z.push_back(path_points[i].pose.position.z);
            tmp_v.push_back(path_points[i].twist.linear.x);
            continue;
          }
          else
          {
            continue;
          }
        }
      }
      tmp_x.push_back(path_points[i].pose.position.x);
      tmp_y.push_back(path_points[i].pose.position.y);
      tmp_z.push_back(path_points[i].pose.position.z);
      tmp_v.push_back(path_points[i].twist.linear.x);
    }
    
    std::vector<double> base_s = horibe_spline::calcEuclidDist(tmp_x, tmp_y);
    std::vector<double> new_s;
    double last_s;
    for(double i = 0.0; 
        i <= base_s.back();
        i += delta_arc_length_for_path_smoothing_)
    {
      new_s.push_back(i);
      last_s = i;
    }
    if(path_points.back().twist.linear.x < 1e-8)
    {
      new_s.push_back(base_s.back());
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
  }
  return true;
}
} // namespace path_planner

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>

#include <lanelet2_extension/utility/message_conversion.h>


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
  // std::cout << "relative p "<<relative_p.x <<" "<<relative_p.y << std::endl;
  // double bottomleft_x = (relative_p.x+ego_costmap_x_length/2)/costmap_resolution;
  // double bottomleft_y = (relative_p.y+ego_costmap_y_width/2)/costmap_resolution;
  // std::cout << "heifht "<< occupancy_grid_info.height << std::endl;
  // std::cout << "width "<< occupancy_grid_info.width << std::endl;
  double resolution = occupancy_grid_info.resolution;
  double map_y_height = occupancy_grid_info.height;
  double map_x_width = occupancy_grid_info.width;
  double map_x_in_image_resolution = relative_p.x/resolution;
  double map_y_in_image_resolution = relative_p.y/resolution;
  double image_x = map_y_height - map_y_in_image_resolution;
  double image_y = map_x_width - map_x_in_image_resolution;
  // double bottomleft_x = (relative_p.x)/costmap_resolution;
  // double bottomleft_y = (relative_p.y)/costmap_resolution;
  // double image_x = ego_costmap_y_width/costmap_resolution - bottomleft_y;
  // double image_y = ego_costmap_x_length/costmap_resolution - bottomleft_x;
  // std::cout << "costmap width "<< ego_costmap_y_width << std::endl;
  // std::cout << "costmap length "<< ego_costmap_x_length << std::endl;
  // std::cout << "image x y "<< image_x << " "<< image_y << std::endl;
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
  
  // double bottomleft_x = ego_costmap_y_width/costmap_resolution - image_point.y;
  // double bottomleft_y = ego_costmap_x_length/costmap_resolution - image_point.x;
  // double relative_x = bottomleft_x*costmap_resolution - ego_costmap_x_length/2;
  // double relative_y = bottomleft_y*costmap_resolution - ego_costmap_y_width/2;

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
  path_pub_ = nh_.advertise<autoware_planning_msgs::Path>("planning/scenario_planning/scenarios/lane_following/motion_planning/avoiding_path", 1, true);
  debug_traj_pub_ = 
   nh_.advertise<autoware_planning_msgs::Trajectory>("/debug_traj", 1, true);
  debug_fixed_traj_pub_ = 
   nh_.advertise<autoware_planning_msgs::Trajectory>("/debug_fixed_traj", 1, true);
    
  
  
  twist_sub_ = private_nh_.subscribe("/current_velocity", 1, 
                           &EBPathPlannerNode::currentVelocityCallback, this);
  map_sub_ = private_nh_.subscribe("/lanelet_map_bin", 10,
                           &EBPathPlannerNode::mapCallback, this);
  route_sub_ = private_nh_.subscribe("/planning/mission_planning/route", 10,
                           &EBPathPlannerNode::routeCallback, this);
  objects_sub_ = private_nh_.subscribe("/perception/prediction/objects", 10,
                           &EBPathPlannerNode::objectsCallback, this);
  // avoid_mode_sub_ = private_nh_.subscribe("/perception/prediction/objects", 10,
  //                          &EBPathPlannerNode::objectsCallback, this);
  private_nh_.param<bool>("enable_velocity_based_cropping", 
                         enable_velocity_based_cropping_,false);
  private_nh_.param<bool>("is_debug_fixing_points_mode", 
                           is_debug_no_fixing_points_mode_,false);
  private_nh_.param<bool>("is_relaying_path_mode", 
                           is_relaying_path_mode_,true);
  private_nh_.param<bool>("use_optimization_when_relaying", 
                           use_optimization_when_relaying_,true);
  private_nh_.param<int>("number_of_fixing_points", 
                          number_of_fixing_points_, 15);
  private_nh_.param<double>("time_for_calculating_velocity_based_distance", 
                    time_for_calculating_velocity_based_distance_, 5);
  private_nh_.param<double>("distance_for_cropping", 
                             distance_for_cropping_, -3);
  private_nh_.param<double>("backward_fixing_distance", 
                             backward_fixing_distance_, 5);
  private_nh_.param<double>("exploring_minumum_radius", 
                             exploring_minimum_radius_, 1.4);
  private_nh_.param<double>("forward_fixing_distance", 
                             forward_fixing_distance_, 15.0);
  private_nh_.param<double>("delta_arc_length", 
                             delta_arc_length_, 0.3);
  private_nh_.param<double>("delta_ego_point_threshold", 
                             delta_ego_point_threshold_, 5);
  private_nh_.param<double>("max_avoiding_objects_velocity_ms", 
                             max_avoiding_objects_velocity_ms_, 0.1);
  modify_reference_path_ptr_ = std::make_unique<ModifyReferencePath>(
    exploring_minimum_radius_,
    backward_fixing_distance_);
  eb_path_smoother_ptr_ = std::make_unique<EBPathSmoother>(
    number_of_fixing_points_,
    exploring_minimum_radius_,
    backward_fixing_distance_,
    forward_fixing_distance_,
    delta_arc_length_);
  previous_optimized_points_ptr_ = 
    std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>();
}

EBPathPlannerNode::~EBPathPlannerNode() {}

void EBPathPlannerNode::callback(const autoware_planning_msgs::Path &input_path_msg,
                                 autoware_planning_msgs::Trajectory &output_trajectory_msg)
{
  // if(!previous_optimized_points_ptr_->empty())
  // {
  //   std::vector<autoware_planning_msgs::TrajectoryPoint> fine_optimized_points2;
  //   generateFineOptimizedPoints(
  //     input_path_msg.points,
  //     *previous_optimized_points_ptr_,
  //     fine_optimized_points2);
  
  //   for (int i = 0; i < previous_optimized_points_ptr_->size(); i++)
  //   {
  //     std::cout << "fixed yaw "<< tf2::getYaw(previous_optimized_points_ptr_->at(i).pose.orientation) << std::endl;
  //   }
  //   output_trajectory_msg.points = fine_optimized_points2;
  //   output_trajectory_msg.header = input_path_msg.header;
  //   // output_trajectory_msg.points = *previous_optimized_points_ptr_;
  //   return;
  // }
  if(!lanelet_map_ptr_||
     !in_route_ptr_ )
  {
    return;
  }
  if(!in_objects_ptr_)
  {
    in_objects_ptr_ = std::make_unique<autoware_perception_msgs::DynamicObjectArray>();
  }
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
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
  cv::Mat clearance_map;
  generateClearanceMap(
    input_path_msg.drivable_area,
    in_objects_ptr_->objects,
    self_pose,
    clearance_map);
    
  std::vector<autoware_planning_msgs::TrajectoryPoint> fixed_optimized_points; 
  generateFixedOptimizedPoints(
      self_pose, 
      previous_optimized_points_ptr_,
      clearance_map,
      input_path_msg.drivable_area.info, 
      fixed_optimized_points);
  geometry_msgs::Pose start_exploring_pose;
  if(fixed_optimized_points.empty())
  {
    ROS_INFO("[EBPathPlanner] No fixing points");
    start_exploring_pose = self_pose;
  }
  else if(fixed_optimized_points.size()>=number_of_fixing_points_)
  {
    // start_exploring_pose = 
    //   fixed_optimized_points
    //     [fixed_optimized_points.size()-number_of_fixing_points_].pose;
    start_exploring_pose = 
      fixed_optimized_points.back().pose;
  }
  else
  {
    start_exploring_pose = fixed_optimized_points.back().pose;
  }
  
  // std::cout << "start exploring pose yaw "<< tf2::getYaw(start_exploring_pose.orientation) << std::endl;
  // if(needReset(*previous_ego_point_ptr_, 
  //              self_pose.position))
  if(needReset(*previous_ego_point_ptr_, 
               self_pose.position,
               clearance_map,
               input_path_msg.drivable_area.info,
               fixed_optimized_points))
  {
    modify_reference_path_ptr_ = std::make_unique<ModifyReferencePath>(
                exploring_minimum_radius_,
                backward_fixing_distance_);
    eb_path_smoother_ptr_ = std::make_unique<EBPathSmoother>(
                number_of_fixing_points_,
                exploring_minimum_radius_,
                backward_fixing_distance_,
                forward_fixing_distance_,
                delta_arc_length_);
    previous_optimized_points_ptr_ = 
      std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>();
    fixed_optimized_points.clear();
    start_exploring_pose = self_pose;
  }
  
  std::vector<geometry_msgs::Point> explored_points;
  geometry_msgs::Point debug_goal_point;
  std::vector<geometry_msgs::Point> debug_rearranged_points;
  modify_reference_path_ptr_->generateModifiedPath(
        self_pose, 
        start_exploring_pose,
        input_path_msg.points,
        in_objects_ptr_->objects,
        *routing_graph_ptr_, 
        *lanelet_map_ptr_,
        *in_route_ptr_,
        explored_points,
        clearance_map, 
        input_path_msg.drivable_area.info,
        debug_goal_point,
        debug_rearranged_points);
  std::vector<geometry_msgs::Point> debug_interpolated_points;
  std::vector<geometry_msgs::Point> debug_cached_explored_points;
  std::vector<geometry_msgs::Point> debug_boundary_points;
  std::vector<geometry_msgs::Point> debug_boundary_lb_points;
  std::vector<geometry_msgs::Point> debug_boundary_ub_points;
  std::vector<geometry_msgs::Point> debug_fixed_optimization_points;
  std::vector<geometry_msgs::Point> debug_variable_optimization_points;
  std::vector<geometry_msgs::Point> debug_constrain_points;
  std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
  eb_path_smoother_ptr_->generateOptimizedPath(
        input_path_msg.points,
        explored_points, 
        start_exploring_pose,
        fixed_optimized_points,
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
  std::vector<autoware_planning_msgs::TrajectoryPoint> merged_optimized_points;
  merged_optimized_points = fixed_optimized_points;
  merged_optimized_points.insert(
    merged_optimized_points.end(),
    optimized_points.begin(), 
    optimized_points.end());
  
  alighWithPathPoints(
    input_path_msg.points,
    merged_optimized_points);
  previous_ego_point_ptr_ = std::make_unique<geometry_msgs::Point>(self_pose.position);
  previous_optimized_points_ptr_ = 
    std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>(merged_optimized_points);
  std::vector<autoware_planning_msgs::TrajectoryPoint> fine_optimized_points;
  generateFineOptimizedPoints(
    input_path_msg.points,
    merged_optimized_points, 
    fine_optimized_points);
  output_trajectory_msg.points = fine_optimized_points;
  output_trajectory_msg.header = input_path_msg.header;
  
  
  std::vector<geometry_msgs::Point> debug_fixed_optimzied_points_used_for_constrain;
  std::vector<geometry_msgs::Point> debug_interpolated_points_used_for_optimization;
  if(is_relaying_path_mode_)
  {
    std::vector<autoware_planning_msgs::TrajectoryPoint> smooth_trajectory_points;
    convertPathToSmoothTrajectory(
      self_pose,
      input_path_msg.points,
      fixed_optimized_points,
      smooth_trajectory_points,
      debug_fixed_optimzied_points_used_for_constrain,
      debug_interpolated_points_used_for_optimization);
    std::cout << "smooth trajectory points "<< smooth_trajectory_points.size() << std::endl;
    // std::cout << "debug fixe "<< debug_fixed_optimzied_points_used_for_constrain.size() << std::endl;
    // std::cout << "debug interpo "<< debug_interpolated_points_used_for_optimization.size() << std::endl;
    alighWithPathPoints(
      input_path_msg.points,
      smooth_trajectory_points);
    previous_optimized_points_ptr_ = 
      std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>
        (smooth_trajectory_points);
    for (int i = 0; i < smooth_trajectory_points.size(); i++)
    {
      // std::cout << "loop final caching traj "<< tf2::getYaw(smooth_trajectory_points[i].pose.orientation) << std::endl;
    }
    std::vector<autoware_planning_msgs::TrajectoryPoint> fine_optimized_points2;
    generateFineOptimizedPoints(
      input_path_msg.points,
      smooth_trajectory_points,
      fine_optimized_points2);
  
    output_trajectory_msg.points = fine_optimized_points2;
    
    
    visualization_msgs::MarkerArray marker_array;
    int unique_id = 0;
    visualization_msgs::Marker smooth_debug_fixed_constrain_marker;
    smooth_debug_fixed_constrain_marker.lifetime = ros::Duration(1.0);
    smooth_debug_fixed_constrain_marker.header = input_path_msg.header;
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
    // smooth_debug_fixed_constrain_marker.color.g = 1.0f;
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
    smooth_interpolated_points.header = input_path_msg.header;
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
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "    total time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;

  std::cout << "KKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKKK" << std::endl;
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
  
  visualization_msgs::Marker debug_fixing_optimized_points_marker;
  debug_fixing_optimized_points_marker.lifetime = ros::Duration(1.0);
  debug_fixing_optimized_points_marker.header = input_path_msg.header;
  debug_fixing_optimized_points_marker.ns = std::string("fixing_optimized_points_marker");
  debug_fixing_optimized_points_marker.action = visualization_msgs::Marker::MODIFY;
  debug_fixing_optimized_points_marker.pose.orientation.w = 1.0;
  debug_fixing_optimized_points_marker.id = unique_id;
  debug_fixing_optimized_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  debug_fixing_optimized_points_marker.scale.x = 1.0f;
  debug_fixing_optimized_points_marker.scale.y = 0.1f;
  debug_fixing_optimized_points_marker.scale.z = 0.1f;
  debug_fixing_optimized_points_marker.color.r = 1.0f;
  debug_fixing_optimized_points_marker.color.g = 1.0f;
  debug_fixing_optimized_points_marker.color.a = 0.999;
  for (int i = 0; i < fixed_optimized_points.size(); i++)
  {
    debug_fixing_optimized_points_marker.points.push_back(
      fixed_optimized_points[i].pose.position);
  }
  if(!debug_fixing_optimized_points_marker.points.empty())
  {
    marker_array.markers.push_back(debug_fixing_optimized_points_marker);
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
  
  for (size_t i = 0; i < fixed_optimized_points.size(); i++)
  {
    visualization_msgs::Marker text_marker;
    text_marker.lifetime = ros::Duration(1.0);
    text_marker.header = input_path_msg.header;
    text_marker.ns = std::string("text_fixed_optimized_points");
    text_marker.action = visualization_msgs::Marker::MODIFY;
    text_marker.pose.orientation.w = 1.0;
    text_marker.id = unique_id;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 0.1f;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.a = 0.999;
    text_marker.text = std::to_string((int)i);
    text_marker.pose.position = fixed_optimized_points[i].pose.position;
    text_marker.pose.orientation.w = 1.0;
    marker_array.markers.push_back(text_marker);
    unique_id++;
    
  }
  markers_pub_.publish(marker_array);
}

void EBPathPlannerNode::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  in_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(msg);
}

void EBPathPlannerNode::mapCallback(
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

void EBPathPlannerNode::routeCallback(
  const autoware_planning_msgs::Route& msg)
{
  in_route_ptr_ = std::make_shared<autoware_planning_msgs::Route>(msg);
}

void EBPathPlannerNode::objectsCallback(
  const autoware_perception_msgs::DynamicObjectArray& msg)
{
  in_objects_ptr_ = std::make_shared<autoware_perception_msgs::DynamicObjectArray>(msg);
}

bool EBPathPlannerNode::needReset(
  const geometry_msgs::Point& previous_ego_point,
  const geometry_msgs::Point& current_ego_point,
  const cv::Mat& clearance_map,
  const nav_msgs::MapMetaData& map_info,
  const std::vector<autoware_planning_msgs::TrajectoryPoint>& fixed_optimized_points)
{
  bool is_need_reset = false;
  //check1
  double dx = previous_ego_point.x - current_ego_point.x;
  double dy = previous_ego_point.y - current_ego_point.y;
  double dist = std::sqrt(dx*dx+dy*dy);
  if(dist > delta_ego_point_threshold_ || is_debug_no_fixing_points_mode_)
  {
    ROS_WARN(
      "[EBPathPlanner] Reset eb path planner since ego pose is moved more than %lf", 
       delta_ego_point_threshold_);
    is_need_reset = true;
  }
  else
  {
    is_need_reset = false;
  }
  
  if(is_need_reset)
  {
    return is_need_reset;
  }
  
  //check2
  int count = 0;
  for(const auto& point: fixed_optimized_points)
  {
    geometry_msgs::Point point_in_image;
    if(tmp1::transformMapToImage(
            point.pose.position, 
            map_info,
            point_in_image))
    {
       float clearance = 
          clearance_map.ptr<float>((int)point_in_image.y)
                                  [(int)point_in_image.x]*map_info.resolution;
       if(clearance < 1e-6)
       {
        //  ROS_ERROR("ocunt %d", count);
         ROS_WARN(
          "[EBPathPlanner] Reset eb path planner since optimized points are outside of drivavle area");
         is_need_reset = true;
         break;
       }
    }
    count++;
  }
  return is_need_reset;
}

bool EBPathPlannerNode::generateFixedOptimizedPoints(
  const geometry_msgs::Pose& ego_pose,
  const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>& previous_optimized_points_ptr,
  const cv::Mat& clearance_map,
  const nav_msgs::MapMetaData& map_info,
  std::vector<autoware_planning_msgs::TrajectoryPoint>& fixed_optimized_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  if(!previous_optimized_points_ptr)
  {
    ROS_WARN("[EBPathPlanner] previous_optimized_points_ptr is nullptr; no fixed points are passed");
    return false;
  }
  double min_dist = 1000000;
  int min_ind = 0;
  for (int i = 0; i < previous_optimized_points_ptr->size(); i++)
  {
    double dx = previous_optimized_points_ptr->at(i).pose.position.x -  ego_pose.position.x; 
    double dy = previous_optimized_points_ptr->at(i).pose.position.y -  ego_pose.position.y; 
    double dist = std::sqrt(dx*dx+dy*dy);
    // std::cout << "dist "<< dist << std::endl;
    if(dist < min_dist)
    {
      min_dist = dist;
      min_ind = i;
    }
  }
  std::cout << "prv opt size "<< previous_optimized_points_ptr->size() << std::endl;
  std::cout << "min ind "<< min_ind << std::endl;
  const double keep_distance = forward_fixing_distance_ + backward_fixing_distance_;
  int forward_fixing_idx = 
    std::min((int)(min_ind+forward_fixing_distance_/delta_arc_length_),
             (int)previous_optimized_points_ptr->size());
  const double backward_keep_distance = 
    keep_distance - (forward_fixing_idx-min_ind)*delta_arc_length_;
  int backward_fixing_idx = 
    std::max((int)(min_ind-backward_keep_distance/delta_arc_length_),
                  0);
         
  for (int i = backward_fixing_idx; i < forward_fixing_idx; i++)
  {
    fixed_optimized_points.push_back(previous_optimized_points_ptr->at(i));
  }     
      
  // int origin_valid_prev_optimized_points_ind = 0;
  // for (int i = 0; i < forward_fixing_idx; i++)
  // {
  //   geometry_msgs::Point point_in_image;
  //   if(tmp1::transformMapToImage(
  //           previous_optimized_points_ptr_->at(i).pose.position, 
  //           map_info,
  //           point_in_image))
  //   {
  //      float clearance = 
  //         clearance_map.ptr<float>((int)point_in_image.y)
  //                                 [(int)point_in_image.x]*map_info.resolution;
  //      origin_valid_prev_optimized_points_ind = i;
  //      if(clearance > 0)
  //      {
  //       //  origin_valid_prev_optimized_points_ind = std::min(i+10, forward_fixing_idx);
  //        break;
  //      }
  //      else
  //      {
  //        ROS_WARN_THROTTLE(1.0, 
  //             "[EBPathPlanner] Discard fixed optimized points since they are out of dirvable area");
  //      }
  //   }
  // }
  // int valid_backward_fixing_idx =  std::max(origin_valid_prev_optimized_points_ind, backward_fixing_idx);
  // valid_backward_fixing_idx = std::min(valid_backward_fixing_idx, forward_fixing_idx);
  // for (int i = valid_backward_fixing_idx; i < forward_fixing_idx; i++)
  // {
  //   fixed_optimized_points.push_back(previous_optimized_points_ptr->at(i));
  // }
  
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

  drivable_area.forEach<uchar>
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
  
  
  
  //--debug
  // cv::Mat tmp;
  // clearance_map.copyTo(tmp);
  // cv::normalize(tmp, tmp, 0, 255, cv::NORM_MINMAX, CV_8UC1);
  
  // geometry_msgs::Point ddd;
  // if(tmp1::transformMapToImage(
  //             debug_ego_pose.position, 
  //             occupancy_grid.info,
  //             ddd ))
  // {
  //   std::cout << "ddd "<<ddd.x <<" "<<ddd.y<< std::endl;
  //   cv::circle(tmp, 
  //               cv::Point(ddd.x, ddd.y), 
  //               10, 
  //               cv::Scalar(0),
  //               1);
  //   geometry_msgs::Point aaa;
  //   tmp1::transformImageToMap(
  //     ddd,
  //     occupancy_grid.info,
  //     aaa);
  //   std::cout << "map " <<aaa.x<<" "<<aaa.y<< std::endl;
  //   std::cout << "pose "<< debug_ego_pose.position.x<< " "<<debug_ego_pose.position.y << std::endl;
  // } 
  
  // cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
  // cv::imshow("image", tmp);
  // cv::waitKey(20);
  //end debugs
}

bool EBPathPlannerNode::convertPathToSmoothTrajectory(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& fixed_optimized_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& smoothed_points,
    std::vector<geometry_msgs::Point>& debug_fixed_optimzied_points_used_for_constrain,
    std::vector<geometry_msgs::Point>& debug_interpolated_points_used_for_optimization)
{
  if(use_optimization_when_relaying_)
  {
    
    std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
    // std::vector<geometry_msgs::Point> debug_fixed_optimzied_points_used_for_constrain;
    // std::vector<geometry_msgs::Point> debug_interpolated_points_used_for_optimization;
    eb_path_smoother_ptr_->generateOptimizedPath(
      ego_pose,
      path_points,
      fixed_optimized_points,
      optimized_points,
      debug_fixed_optimzied_points_used_for_constrain,
      debug_interpolated_points_used_for_optimization);
      
    autoware_planning_msgs::Trajectory tmp_traj;
    tmp_traj.points = optimized_points;
    debug_traj_pub_.publish(tmp_traj);
    
    autoware_planning_msgs::Trajectory tmp_fixed_traj;
    tmp_fixed_traj.points = fixed_optimized_points;
    debug_fixed_traj_pub_.publish(tmp_fixed_traj);
    
    std::vector<autoware_planning_msgs::TrajectoryPoint> merged_optimized_points;
    // merged_optimized_points = fixed_optimized_points;
    // merged_optimized_points.insert(merged_optimized_points.end(),
    //                       optimized_points.begin(),
    //                       optimized_points.end());
    merged_optimized_points = optimized_points;
              
    if(!fixed_optimized_points.empty()&& !optimized_points.empty())
    {
      // std::cout << "last fixed yaw "<<tf2::getYaw(fixed_optimized_points.back().pose.orientation) << std::endl;            
      // std::cout << "first optimized yaw "<<tf2::getYaw(optimized_points.front().pose.orientation) << std::endl;            
      double dy = optimized_points.front().pose.position.y - fixed_optimized_points.back().pose.position.y;
      double dx = optimized_points.front().pose.position.x - fixed_optimized_points.back().pose.position.x;
      // std::cout << "yaw from back to front "<<std::atan2(dy, dx) << std::endl;            
      
    }
    std::vector<double> tmp_x;
    std::vector<double> tmp_y;
    std::vector<double> tmp_z;
    std::vector<double> tmp_v;
    for(size_t i = 0; i <  merged_optimized_points.size(); i++)
    {
      //backward check
      if(i > 1)
      {
        double dx1 = merged_optimized_points[i].pose.position.x - 
                    merged_optimized_points[i-1].pose.position.x;
        double dy1 = merged_optimized_points[i].pose.position.y - 
                    merged_optimized_points[i-1].pose.position.y;
        double dx2 = merged_optimized_points[i-1].pose.position.x - 
                    merged_optimized_points[i-2].pose.position.x;
        double dy2 = merged_optimized_points[i-1].pose.position.y - 
                    merged_optimized_points[i-2].pose.position.y;
        double inner_product = dx1*dx2 + dy1*dy2;
        if(inner_product < 0)
        {
          ROS_INFO("Path points might go backward");
        }
      }
      
      //resolution check
      // if(i > 0)
      // {
      //   double dx = merged_optimized_points[i].pose.position.x - 
      //               merged_optimized_points[i-1].pose.position.x;
      //   double dy = merged_optimized_points[i].pose.position.y - 
      //               merged_optimized_points[i-1].pose.position.y;
      //   double dist = std::sqrt(dx*dx + dy*dy); 
      //   if(dist < 0.1)
      //   {
      //     if(merged_optimized_points[i].twist.linear.x < 
      //       merged_optimized_points[i-1].twist.linear.x)
      //     {
      //       //replace point with lower velocity
      //       tmp_x.pop_back();
      //       tmp_y.pop_back();
      //       tmp_z.pop_back();
      //       tmp_v.pop_back();
      //       tmp_x.push_back(merged_optimized_points[i].pose.position.x);
      //       tmp_y.push_back(merged_optimized_points[i].pose.position.y);
      //       tmp_z.push_back(merged_optimized_points[i].pose.position.z);
      //       tmp_v.push_back(merged_optimized_points[i].twist.linear.x);
      //       continue;
      //     }
      //     else
      //     {
      //       continue;
      //     }
      //   }
      // }
      tmp_x.push_back(merged_optimized_points[i].pose.position.x);
      tmp_y.push_back(merged_optimized_points[i].pose.position.y);
      tmp_z.push_back(merged_optimized_points[i].pose.position.z);
      tmp_v.push_back(merged_optimized_points[i].twist.linear.x);
    }
    // std::cout << "tmpxy "<< tmp_x.back()<<" "<<tmp_y.back() << std::endl;
    
    std::vector<double> base_s = horibe_spline::calcEuclidDist(tmp_x, tmp_y);
    std::vector<double> new_s;
    for(double i = 0.0; 
        i <= base_s.back();
        i += delta_arc_length_)
    {
      new_s.push_back(i);
    }
    // new_s.push_back(base_s.back());
    
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
        // msgConversionFromPath2Trajectory(path_pointst_trajectory_msg); 
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
      // std::cout << "merged yaw "<< yaw << std::endl;
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
    // std::cout << "last smooth "<< smoothed_points.back().pose.position.x<< " "<<smoothed_points.back().pose.position.y << std::endl;
    
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
        i += delta_arc_length_)
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
        // msgConversionFromPath2Trajectory(path_pointst_trajectory_msg); 
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

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>

#include <lanelet2_extension/utility/message_conversion.h>


#include <tf2/utils.h>

#include <memory>
#include <chrono>
#include <algorithm>

#include "eb_path_planner/modify_reference_path.h"
#include "eb_path_planner/eb_path_smoother.h"
#include "eb_path_planner/node.hpp"

namespace path_planner
{
EBPathPlannerNode::EBPathPlannerNode()
    : nh_(),
      private_nh_("~")
{
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("eb_path_planner_marker", 1, true);
  path_pub_ = nh_.advertise<autoware_planning_msgs::Path>("planning/scenario_planning/scenarios/lane_following/motion_planning/avoiding_path", 1, true);
  
  twist_sub_ = private_nh_.subscribe("/current_velocity", 1, 
                           &EBPathPlannerNode::currentVelocityCallback, this);
  map_sub_ = private_nh_.subscribe("/lanelet_map_bin", 10,
                           &EBPathPlannerNode::mapCallback, this);
  route_sub_ = private_nh_.subscribe("/planning/mission_planning/route", 10,
                           &EBPathPlannerNode::routeCallback, this);
  objects_sub_ = private_nh_.subscribe("/perception/prediction/objects", 10,
                           &EBPathPlannerNode::objectsCallback, this);
  private_nh_.param<bool>("enable_velocity_based_cropping", 
                         enable_velocity_based_cropping_,false);
  private_nh_.param<int>("num_lookup_lanelet_for_drivealble_area", 
                         num_lookup_lanelet_for_drivealble_area_,4);
  private_nh_.param<double>("time_for_calculating_velocity_based_distance", 
                    time_for_calculating_velocity_based_distance_, 5);
  private_nh_.param<double>("distance_for_cropping", 
                             distance_for_cropping_, -3);
  private_nh_.param<double>("backward_distance", 
                             backward_distance_, 5);
  private_nh_.param<double>("exploring_minumum_radius", 
                             exploring_minimum_radius_, 1.4);
  private_nh_.param<double>("fixing_distance", 
                             fixing_distance_, 20.0);
  private_nh_.param<double>("sampling_resolution", 
                             sampling_resolution_, 0.1);
  private_nh_.param<double>("delta_ego_point_threshold_", 
                             delta_ego_point_threshold_, 5);
  modify_reference_path_ptr_ = std::make_unique<ModifyReferencePath>(
    num_lookup_lanelet_for_drivealble_area_,
    exploring_minimum_radius_,
    backward_distance_);
  eb_path_smoother_ptr_ = std::make_unique<EBPathSmoother>(
    exploring_minimum_radius_,
    backward_distance_,
    fixing_distance_,
    sampling_resolution_);
}

EBPathPlannerNode::~EBPathPlannerNode() {}

void EBPathPlannerNode::callback(const autoware_planning_msgs::Path &input_path_msg,
                                 autoware_planning_msgs::Trajectory &output_trajectory_msg)
{
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
  
  if(needReset(*previous_ego_point_ptr_, self_pose.position))
  {
    modify_reference_path_ptr_ = std::make_unique<ModifyReferencePath>(
                num_lookup_lanelet_for_drivealble_area_,
                exploring_minimum_radius_,
                backward_distance_);
    eb_path_smoother_ptr_ = std::make_unique<EBPathSmoother>(
                exploring_minimum_radius_,
                backward_distance_,
                fixing_distance_,
                sampling_resolution_);
  }
  std::vector<geometry_msgs::Point> debug_points;
  geometry_msgs::Point debug_goal_point;
  modify_reference_path_ptr_->generateModifiedPath(
        self_pose, 
        input_path_msg.points,
        in_objects_ptr_->objects,
        *routing_graph_ptr_, 
        *lanelet_map_ptr_,
        *in_route_ptr_,
        debug_points, 
        debug_goal_point);
  std::vector<geometry_msgs::Point> debug_interpolated_points;
  std::vector<geometry_msgs::Point> debug_cached_explored_points;
  std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
  eb_path_smoother_ptr_->generateOptimizedPath(
        input_path_msg.points,
        debug_points, 
        self_pose, 
        debug_interpolated_points,
        debug_cached_explored_points,
        optimized_points);
  
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "    total time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  output_trajectory_msg.points = optimized_points;
  output_trajectory_msg.header = input_path_msg.header;
  previous_ego_point_ptr_ = std::make_unique<geometry_msgs::Point>(self_pose.position);

  //debug; marker array
  visualization_msgs::MarkerArray marker_array;
  int unique_id = 0;

  // visualize cubic spline point
  visualization_msgs::Marker debug_cubic_spline;
  debug_cubic_spline.lifetime = ros::Duration(1.0);
  debug_cubic_spline.header = input_path_msg.header;
  debug_cubic_spline.ns = std::string("debug_explored_points");
  debug_cubic_spline.action = visualization_msgs::Marker::MODIFY;
  debug_cubic_spline.pose.orientation.w = 1.0;
  debug_cubic_spline.id = unique_id;
  debug_cubic_spline.type = visualization_msgs::Marker::SPHERE_LIST;
  debug_cubic_spline.scale.x = 1.0f;
  debug_cubic_spline.scale.y = 0.1f;
  debug_cubic_spline.scale.z = 0.1f;
  debug_cubic_spline.color.g = 1.0f;
  debug_cubic_spline.color.a = 0.999;
  for(const auto& point: debug_points)
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
  debug_goal_point_marker.ns = std::string("debug_goal_point_marker");
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
  
  // visualize cubic spline point
  visualization_msgs::Marker debug_interpolated_points_marker;
  debug_interpolated_points_marker.lifetime = ros::Duration(1.0);
  debug_interpolated_points_marker.header = input_path_msg.header;
  debug_interpolated_points_marker.ns = std::string("debug_interpolated_points_marker");
  debug_interpolated_points_marker.action = visualization_msgs::Marker::MODIFY;
  debug_interpolated_points_marker.pose.orientation.w = 1.0;
  debug_interpolated_points_marker.id = unique_id;
  debug_interpolated_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  debug_interpolated_points_marker.scale.x = 1.0f;
  debug_interpolated_points_marker.scale.y = 0.1f;
  debug_interpolated_points_marker.scale.z = 0.1f;
  debug_interpolated_points_marker.color.g = 1.0f;
  debug_interpolated_points_marker.color.a = 0.999;
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
  debug_optimized_points_marker.ns = std::string("debug_optimized_points_marker");
  debug_optimized_points_marker.action = visualization_msgs::Marker::MODIFY;
  debug_optimized_points_marker.pose.orientation.w = 1.0;
  debug_optimized_points_marker.id = unique_id;
  debug_optimized_points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
  debug_optimized_points_marker.scale.x = 1.0f;
  debug_optimized_points_marker.scale.y = 0.1f;
  debug_optimized_points_marker.scale.z = 0.1f;
  debug_optimized_points_marker.color.g = 1.0f;
  debug_optimized_points_marker.color.a = 0.999;
  for (size_t i = fixing_distance_/sampling_resolution_; i < optimized_points.size(); i++)
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
  debug_fixing_optimized_points_marker.ns = std::string("debug_fixing_optimized_points_marker");
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
  int tmp_max_ind = std::min((int)std::ceil(fixing_distance_/sampling_resolution_), 
                             (int)optimized_points.size());
  for (int i = 0; i < tmp_max_ind; i++)
  {
    debug_fixing_optimized_points_marker.points.push_back(optimized_points[i].pose.position);
  }
  if(!debug_fixing_optimized_points_marker.points.empty())
  {
    marker_array.markers.push_back(debug_fixing_optimized_points_marker);
  }
  unique_id++;
  
  for (size_t i = 0; i < debug_points.size(); i++)
  {
    visualization_msgs::Marker text_marker;
    text_marker.lifetime = ros::Duration(1.0);
    text_marker.header = input_path_msg.header;
    text_marker.ns = std::string("number_of_explored_points");
    text_marker.action = visualization_msgs::Marker::MODIFY;
    text_marker.pose.orientation.w = 1.0;
    text_marker.id = unique_id;
    text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    text_marker.scale.z = 1.0f;
    text_marker.color.r = 1.0f;
    text_marker.color.g = 1.0f;
    text_marker.color.a = 0.999;
    text_marker.text = std::to_string((int)i);
    text_marker.pose.position = debug_points[i];
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
  const geometry_msgs::Point& current_ego_point)
{
  double dx = previous_ego_point.x - current_ego_point.x;
  double dy = previous_ego_point.y - current_ego_point.y;
  double dist = std::sqrt(dx*dx+dy*dy);
  if(dist > delta_ego_point_threshold_)
  {
    ROS_WARN("[EBPathPlanner] Reset eb path planner");
    return true;
  }
  else
  {
    return false;
  }
}
} // namespace path_planner
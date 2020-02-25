
#include <vector>
#include <chrono>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/MapMetaData.h>
#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>

#include <ros/console.h>
#include <tf2/utils.h>

#include <opencv2/opencv.hpp>

#include "workspace.h"
#include "osqp.h"
#include "auxil.h"

#include "eb_path_planner/eb_path_smoother.h"
#include "eb_path_planner/util.h"

EBPathSmoother::EBPathSmoother(
  double exploring_minimum_radius,
  double backward_fixing_distance,
  double fixing_distance,
  double delta_arc_length_for_path_smoothing,
  double delta_arc_length_for_explored_points):
number_of_sampling_points_(100),
number_of_diff_optimization_points_for_cold_start_(40),
exploring_minimum_radius_(exploring_minimum_radius),
backward_fixing_distance_(backward_fixing_distance),
fixing_distance_(fixing_distance),
delta_arc_length_for_path_smoothing_(delta_arc_length_for_path_smoothing),
delta_arc_length_for_explored_points_(delta_arc_length_for_explored_points),
loose_constrain_disntance_(0.2),
tighten_constrain_disntance_(0.2)
{
  cold_start(&workspace);
}

EBPathSmoother::~EBPathSmoother(){}
  
bool EBPathSmoother::preprocessExploredPoints(
    const std::vector<geometry_msgs::Point>& explored_points,
    const geometry_msgs::Pose& ego_pose,
    std::vector<double>& interpolated_x,
    std::vector<double>& interpolated_y,
    std::vector<geometry_msgs::Point>& interpolated_points,
    int& farrest_idx,
    std::vector<geometry_msgs::Point>& debug_interpolated_points)
{
   
  if(explored_points.empty() || 
     explored_points.size()==1)
  {
    ROS_WARN_THROTTLE(5.0, "[EBPathPlanner] Almost no explored points; Skip optimization");
    return false;
  }
  
  
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(size_t i = 0; i <  explored_points.size(); i++)
  {
    tmp_x.push_back(explored_points[i].x);
    tmp_y.push_back(explored_points[i].y);
  }
  // std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, 
        delta_arc_length_for_explored_points_, interpolated_points);
      
  debug_interpolated_points = interpolated_points;
  farrest_idx = std::min((int)(number_of_sampling_points_-1),
                         (int)(interpolated_points.size()-1));
  return true;
}
  
bool EBPathSmoother::preprocessPathPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const geometry_msgs::Point& start_point,
    const geometry_msgs::Point& goal_point,
    const geometry_msgs::Pose& ego_pose,
    std::vector<double>& interpolated_x,
    std::vector<double>& interpolated_y,
    std::vector<geometry_msgs::Point>& interpolated_points,
    int& farrest_idx,
    std::vector<geometry_msgs::Point>& debug_interpolated_points)
{
  
  
  if(path_points.empty() || 
     path_points.size()==1)
  {
    ROS_WARN("[EBPathPlanner] Almost no path points");
    return false;
  }
  
  int start_ind = -1;
  int goal_ind = -1;
  // for(const auto& point: path_points)
  for (int i = 0; i < path_points.size(); i++)
  {
    double dx1 = path_points[i].pose.position.x - start_point.x;
    double dy1 = path_points[i].pose.position.y - start_point.y;
    double dist1 = std::sqrt(dx1*dx1+dy1*dy1);
    if(dist1 < 1e-4)
    {
      start_ind = i;
    }
    double dx2 = path_points[i].pose.position.x - goal_point.x;
    double dy2 = path_points[i].pose.position.y - goal_point.y;
    double dist2 = std::sqrt(dx2*dx2+dy2*dy2);
    if(dist2 < 1e-4)
    {
      goal_ind = i;
    }
  }
  if(start_ind == -1 || goal_ind == -1)
  {
    ROS_WARN("Could not find correspongin path points with start and/or goal in preprocessPathPoints");
  }
  
  
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(size_t i = start_ind;
             i <=  goal_ind; i++)
  {
    tmp_x.push_back(path_points[i].pose.position.x);
    tmp_y.push_back(path_points[i].pose.position.y);
  }
  util::interpolate2DPoints(tmp_x, tmp_y, 
          delta_arc_length_for_path_smoothing_, interpolated_points);
  
  debug_interpolated_points = interpolated_points;
  farrest_idx = std::min((int)(number_of_sampling_points_-1),
                         (int)(interpolated_points.size()-1));
  return true;
}
  
bool EBPathSmoother::generateOptimizedExploredPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
    const std::vector<geometry_msgs::Point>& explored_points,
    const geometry_msgs::Pose& ego_pose,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info,
    std::vector<geometry_msgs::Point>& debug_interpolated_points,
    std::vector<geometry_msgs::Point>& debug_constrain_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  std::vector<double> interpolated_x;
  std::vector<double> interpolated_y;
  std::vector<geometry_msgs::Point> interpolated_points;
  int farrest_idx_from_ego_pose = 0;
  bool is_preprocess_success = 
    preprocessExploredPoints(explored_points, 
                             ego_pose,
                             interpolated_x,
                             interpolated_y,
                             interpolated_points,
                             farrest_idx_from_ego_pose,
                             debug_interpolated_points);
  if(!is_preprocess_success)
  {
    return false;
  }           
  
  Mode qp_mode = Mode::Avoidance;
  updateQPConstrain(interpolated_points, 
                    farrest_idx_from_ego_pose,
                    clearance_map,
                    map_info,
                    qp_mode,
                    debug_constrain_points);
  
  solveQP(qp_mode);
  
  int number_of_optimized_points = 
    std::min(farrest_idx_from_ego_pose, number_of_sampling_points_-1);
  optimized_points = 
    generatePostProcessedTrajectoryPoints(number_of_optimized_points,ego_pose);
  
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "  optimization time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
} 

bool EBPathSmoother::generateOptimizedPath(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
    const geometry_msgs::Point& start_path_point,
    const geometry_msgs::Point& goal_path_point,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points,
    std::vector<geometry_msgs::Point>& debug_constrain_points,
    std::vector<geometry_msgs::Point>& debug_interpolated_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  std::vector<double> interpolated_x;
  std::vector<double> interpolated_y;
  std::vector<geometry_msgs::Point> interpolated_points;
  int farrest_idx_from_start_point;
  bool is_preprocess_success = 
    preprocessPathPoints(path_points, 
                        start_path_point,
                        goal_path_point,
                        ego_pose,
                        interpolated_x,
                        interpolated_y,
                        interpolated_points,
                        farrest_idx_from_start_point,
                        debug_interpolated_points);
  
  if(!is_preprocess_success)
  {
    return false;
  }           
  Mode qp_mode = Mode::LaneFollowing;
  //TODO: something better way?
  cv::Mat dummy_clearance_map;
  nav_msgs::MapMetaData dummy_map_info;
  updateQPConstrain(interpolated_points, 
                    farrest_idx_from_start_point,
                    dummy_clearance_map,
                    dummy_map_info,
                    qp_mode,
                    debug_constrain_points);
  
  solveQP(qp_mode);
  
  int number_of_optimized_points = 
    std::min(farrest_idx_from_start_point, number_of_sampling_points_-1);
  optimized_points = 
    generatePostProcessedTrajectoryPoints(number_of_optimized_points, ego_pose);
  
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "  optimization time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
} 

void EBPathSmoother::solveQP(const Mode qp_mode)
{
  if(qp_mode == Mode::Avoidance)
  {
    osqp_update_eps_rel(&workspace, 1e-5f);
  }
  else
  {
    osqp_update_eps_rel(&workspace, 1e-6f);
  }
  osqp_update_eps_abs(&workspace, 1e-2f);
  osqp_update_alpha(&workspace, 1.6);
  osqp_update_max_iter(&workspace, 16000);
  osqp_solve(&workspace);
  // printf("Status:                %s\n", (&workspace)->info->status);
  // printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  // printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
}

void EBPathSmoother::updateQPConstrain(
  const std::vector<geometry_msgs::Point>& interpolated_points,
  const int farrest_point_idx,
  const cv::Mat& clearance_map,
  const nav_msgs::MapMetaData& map_info,
  const Mode qp_optimization_mode,
  std::vector<geometry_msgs::Point>& debug_constrain_points)
{
  double lower_bound[number_of_sampling_points_ * 2];
  double upper_bound[number_of_sampling_points_ * 2];
  
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if(i==0)
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    }
    else if (i == farrest_point_idx - 1 )//second last x
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    }
    else if (i == farrest_point_idx )//last x
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    }
    else if(i > farrest_point_idx)
    {
      lower_bound[i] = interpolated_points[farrest_point_idx].x;
      upper_bound[i] = interpolated_points[farrest_point_idx].x;
    }
    else
    {
      if(qp_optimization_mode == Mode::Avoidance)
      {
        geometry_msgs::Point interpolated_p_in_image;
        float clearance;
        if(util::transformMapToImage(
                              interpolated_points[i], 
                              map_info,
                              interpolated_p_in_image))
        {
          clearance = 
            clearance_map.ptr<float>((int)interpolated_p_in_image.y)
                                    [(int)interpolated_p_in_image.x]*
                                    map_info.resolution;
        }
        else
        {
          clearance = 0.5;
        }
        float diff = 
          std::fmax(clearance - exploring_minimum_radius_ - tighten_constrain_disntance_,
                    loose_constrain_disntance_);
        lower_bound[i] = interpolated_points[i].x - diff;
        upper_bound[i] = interpolated_points[i].x + diff;
      }
      else
      {
        lower_bound[i] = interpolated_points[i].x - 0.2;
        upper_bound[i] = interpolated_points[i].x + 0.2;
      }
    }
  }
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if (i == 0)//initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
    }
    else if (i == farrest_point_idx - 1)//second last x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
    }
    else if (i == farrest_point_idx)//last x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
    }
    else if(i >= farrest_point_idx)
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_points[farrest_point_idx].y;
      upper_bound[i+number_of_sampling_points_] = interpolated_points[farrest_point_idx].y;
    }
    else
    {
      if(qp_optimization_mode == Mode::Avoidance)
      {
        geometry_msgs::Point interpolated_p = interpolated_points[i];
        geometry_msgs::Point interpolated_p_in_image;
        float clearance;
        if(util::transformMapToImage(
                              interpolated_points[i], 
                              map_info,
                              interpolated_p_in_image))
        {
          clearance = 
            clearance_map.ptr<float>
              ((int)interpolated_p_in_image.y)
              [(int)interpolated_p_in_image.x]*
              map_info.resolution;
        }
        else
        {
          clearance = 0.5;
        }
        float diff = 
          std::fmax(clearance - exploring_minimum_radius_ - tighten_constrain_disntance_,
                    loose_constrain_disntance_);
        lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y - diff;
        upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y + diff;
        
        interpolated_p.z = diff;
        debug_constrain_points.push_back(interpolated_p);
      }
      else
      {
        lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y - 0.2;
        upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y + 0.2;
      }
      
    }
  }
  osqp_update_bounds(&workspace, lower_bound, upper_bound);
}

std::vector<autoware_planning_msgs::TrajectoryPoint> 
    EBPathSmoother::generatePostProcessedTrajectoryPoints(
      const int number_of_optimized_points,
      const geometry_msgs::Pose& ego_pose)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(size_t i = 0; i <=  number_of_optimized_points; i++)
  {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = 
      workspace.solution->x[i];
    tmp_point.pose.position.y = 
      workspace.solution->x[i + number_of_sampling_points_];
    if(i>0)
    {
      double dx = tmp_point.pose.position.x - workspace.solution->x[i-1];
      double dy = tmp_point.pose.position.y - workspace.solution->x[i+number_of_sampling_points_-1];
    }
    tmp_x.push_back(tmp_point.pose.position.x);
    tmp_y.push_back(tmp_point.pose.position.y);
  }
  std::vector<geometry_msgs::Point> post_interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, 
      delta_arc_length_for_path_smoothing_, post_interpolated_points);
  
  std::vector<autoware_planning_msgs::TrajectoryPoint> optimized_points;
  for (int i = 0; i < post_interpolated_points.size(); i++)
  {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position = post_interpolated_points[i];
    tmp_point.pose.position.z = ego_pose.position.z;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    if(i==post_interpolated_points.size()-1)
    {
      double dx = post_interpolated_points[i].x - 
                  post_interpolated_points[i-1].x; 
      double dy = post_interpolated_points[i].y - 
                  post_interpolated_points[i-1].y; 
      yaw = std::atan2(dy, dx);
    }
    else
    {
      double dx = post_interpolated_points[i+1].x - 
                  post_interpolated_points[i].x; 
      double dy = post_interpolated_points[i+1].y - 
                  post_interpolated_points[i].y;
      yaw = std::atan2(dy, dx); 
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    tmp_point.pose.orientation = tf2::toMsg(quaternion);
    optimized_points.push_back(tmp_point);
  }
  return optimized_points;
}
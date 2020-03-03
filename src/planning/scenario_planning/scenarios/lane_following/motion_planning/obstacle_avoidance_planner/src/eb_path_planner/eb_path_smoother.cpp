
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
loose_constrain_disntance_(0.1),
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
  
std::vector<autoware_planning_msgs::TrajectoryPoint>  
  EBPathSmoother::generateOptimizedExploredPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
    const std::vector<geometry_msgs::Point>& fixed_points,
    const std::vector<geometry_msgs::Point>& non_fixed_points,
    const geometry_msgs::Pose& ego_pose,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info,
    std::vector<geometry_msgs::Point>& debug_interpolated_points,
    std::vector<geometry_msgs::Point>& debug_constrain_points)
{
  std::vector<geometry_msgs::Point> candidate_points;
  for (int i = 0; i < fixed_points.size(); i++)
  {
    candidate_points.push_back(fixed_points[i]);
  }
  for (int i = 0; i < non_fixed_points.size(); i++)
  {
    candidate_points.push_back(non_fixed_points[i]);
  }
  if(candidate_points.empty())
  {
    ROS_WARN("Need to be checked, empty candidate points");
    std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points;
    for(const auto& point: path_points)
    {
      autoware_planning_msgs::TrajectoryPoint tmp_point;
      tmp_point.pose = point.pose;
      tmp_point.twist = point.twist;
      traj_points.push_back(tmp_point);
    }
    return traj_points;
  }
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (size_t i = 0; i < candidate_points.size(); i++)
  {
    tmp_x.push_back(candidate_points[i].x);
    tmp_y.push_back(candidate_points[i].y);
  }
  std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length_for_explored_points_, interpolated_points);
  debug_interpolated_points = interpolated_points;
  // std::cout << "explore fixed size "<< fixed_points.size() << std::endl;
  // std::cout << "explore non fixed size "<< non_fixed_points.size() << std::endl;
  
  int farrest_idx_from_start_point = 
    std::min((int)(number_of_sampling_points_-1),
             (int)(interpolated_points.size()-1));
  int num_fixed_points = 0;
  if(!fixed_points.empty() && !interpolated_points.empty())
  {
    double min_dist = 9999999;
    // int nearest_i = 0;
    for (int i = 0; i < interpolated_points.size(); i++)
    {
      double dx = interpolated_points[i].x - fixed_points.back().x;    
      double dy = interpolated_points[i].y - fixed_points.back().y;
      double dist = std::sqrt(dx*dx+dy*dy);   
      double dx1 = 0; 
      double dy1 = 0; 
      if(i == interpolated_points.size()-1)
      {
        dx1 = interpolated_points[i].x - interpolated_points[i-1].x; 
        dy1 = interpolated_points[i].y - interpolated_points[i-1].y; 
      }
      else
      {
        dx1 = interpolated_points[i+1].x - interpolated_points[i].x; 
        dy1 = interpolated_points[i+1].y - interpolated_points[i].y; 
      }
      double ip = dx*dx1+dy*dy1;
      if(dist < min_dist && ip < 0)
      {
        min_dist = dist;
        num_fixed_points = i;
      }
    }
  }
  //make buffer between fixed poins and non-fixed poins
  num_fixed_points = std::max(num_fixed_points - 5, 0);
  num_fixed_points = std::min(num_fixed_points, farrest_idx_from_start_point);
  // std::cout << "farrest idx  "<< farrest_idx_from_start_point << std::endl;
  // std::cout << "num points "<< farrest_idx_from_start_point << std::endl;
  
  Mode qp_mode = Mode::Avoidance;
  updateQPConstrain(interpolated_points, 
                    farrest_idx_from_start_point,
                    num_fixed_points,
                    clearance_map,
                    map_info,
                    qp_mode,
                    debug_constrain_points);
  solveQP(qp_mode);
  
  std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points; 
  for(size_t i = 0; i <=  farrest_idx_from_start_point; i++)
  {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = 
      workspace.solution->x[i];
    tmp_point.pose.position.y = 
      workspace.solution->x[i + number_of_sampling_points_];
    tmp_point.pose.position.z = 0;
    traj_points.push_back(tmp_point);
  }
  for (int i = 0; i < traj_points.size(); i++)
  {
    double yaw = 0;
    if(i==traj_points.size()-1)
    {
      double dx = traj_points[i].pose.position.x - 
                  traj_points[i-1].pose.position.x; 
      double dy = traj_points[i].pose.position.y - 
                  traj_points[i-1].pose.position.y; 
      yaw = std::atan2(dy, dx);
    }
    else
    {
      double dx = traj_points[i+1].pose.position.x - 
                  traj_points[i].pose.position.x; 
      double dy = traj_points[i+1].pose.position.y - 
                  traj_points[i].pose.position.y;
      yaw = std::atan2(dy, dx); 
    }
    tf2::Quaternion quaternion;
    double roll = 0;
    double pitch = 0;
    quaternion.setRPY( roll, pitch, yaw );
    traj_points[i].pose.orientation = tf2::toMsg(quaternion);
  }
  return traj_points;
} 

std::vector<autoware_planning_msgs::TrajectoryPoint> 
    EBPathSmoother::generateOptimizedPoints(
      const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
      const std::vector<geometry_msgs::Pose>& fixed_points,
      const std::vector<geometry_msgs::Pose>& non_fixed_points,
      std::vector<geometry_msgs::Point>& debug_interpolated,
      std::vector<geometry_msgs::Point>& debug_constrain)
{
  //TODO: consider interface aware implementatpn pose or point?
  std::vector<geometry_msgs::Point> candidate_points;
  for (int i = 0; i < fixed_points.size(); i++)
  {
    candidate_points.push_back(fixed_points[i].position);
  }
  for (int i = 0; i < non_fixed_points.size(); i++)
  {
    candidate_points.push_back(non_fixed_points[i].position);
  }
  if(candidate_points.empty())
  {
    ROS_WARN("Need to be checked, empty candidate points");
    std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points;
    for(const auto& point: path_points)
    {
      autoware_planning_msgs::TrajectoryPoint tmp_point;
      tmp_point.pose = point.pose;
      tmp_point.twist = point.twist;
      traj_points.push_back(tmp_point);
    }
    return traj_points;
  }
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for (size_t i = 0; i < candidate_points.size(); i++)
  {
    tmp_x.push_back(candidate_points[i].x);
    tmp_y.push_back(candidate_points[i].y);
  }
  std::vector<geometry_msgs::Point> interpolated_points;
  util::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length_for_path_smoothing_, interpolated_points);
  debug_interpolated = interpolated_points;
  
  
  Mode qp_mode = Mode::LaneFollowing;
  //interpolate point enough to have at least number of sampling points
  int farrest_idx_from_start_point = 
    std::min((int)(number_of_sampling_points_-1),
             (int)(interpolated_points.size()-1));
  // int num_fixed_points = std::max((int)fixed_points.size()- 3 -1, 0);
  // num_fixed_points = std::min(num_fixed_points, farrest_idx_from_start_point);
  int num_fixed_points = 0;
  if(!fixed_points.empty() && !interpolated_points.empty())
  {
    double min_dist = 9999999;
    // int nearest_i = 0;
    for (int i = 0; i < interpolated_points.size(); i++)
    {
      double dx = interpolated_points[i].x - fixed_points.back().position.x;    
      double dy = interpolated_points[i].y - fixed_points.back().position.y;
      double dist = std::sqrt(dx*dx+dy*dy);   
      double dx1 = 0; 
      double dy1 = 0; 
      if(i == interpolated_points.size()-1)
      {
        dx1 = interpolated_points[i].x - interpolated_points[i-1].x; 
        dy1 = interpolated_points[i].y - interpolated_points[i-1].y; 
      }
      else
      {
        dx1 = interpolated_points[i+1].x - interpolated_points[i].x; 
        dy1 = interpolated_points[i+1].y - interpolated_points[i].y; 
      }
      double ip = dx*dx1+dy*dy1;
      if(dist < min_dist && ip < 0)
      {
        min_dist = dist;
        num_fixed_points = i;
      }
    }
  }
  //make buffer between fixed poins and non-fixed poins
  num_fixed_points = std::max(num_fixed_points - 5, 0);
  num_fixed_points = std::min(num_fixed_points, farrest_idx_from_start_point);
  
  // std::cout << "interpolated size "<< interpolated_points.size() << std::endl;
  // std::cout << "path fixed size "<< fixed_points.size() << std::endl;
  // std::cout << "path non fixed size "<< non_fixed_points.size() << std::endl;
  // std::cout << "num fixed points "<< num_fixed_points << std::endl;
  
  // std::cout << "farrest idx  "<< farrest_idx_from_start_point << std::endl;
  updateQPConstrain(interpolated_points,
                    num_fixed_points,
                    farrest_idx_from_start_point,
                    debug_constrain);
  solveQP(qp_mode);
  
  
  std::vector<autoware_planning_msgs::TrajectoryPoint> traj_points; 
  for(size_t i = 0; i <=  farrest_idx_from_start_point; i++)
  {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = 
      workspace.solution->x[i];
    tmp_point.pose.position.y = 
      workspace.solution->x[i + number_of_sampling_points_];
    tmp_point.pose.position.z = 0;
    traj_points.push_back(tmp_point);
  }
  for (int i = 0; i < traj_points.size(); i++)
  {
    double yaw = 0;
    if(i==traj_points.size()-1)
    {
      double dx = traj_points[i].pose.position.x - 
                  traj_points[i-1].pose.position.x; 
      double dy = traj_points[i].pose.position.y - 
                  traj_points[i-1].pose.position.y; 
      yaw = std::atan2(dy, dx);
    }
    else
    {
      double dx = traj_points[i+1].pose.position.x - 
                  traj_points[i].pose.position.x; 
      double dy = traj_points[i+1].pose.position.y - 
                  traj_points[i].pose.position.y;
      yaw = std::atan2(dy, dx); 
    }
    tf2::Quaternion quaternion;
    double roll = 0;
    double pitch = 0;
    quaternion.setRPY( roll, pitch, yaw );
    traj_points[i].pose.orientation = tf2::toMsg(quaternion);
  }
  return traj_points;
}


void EBPathSmoother::solveQP(const Mode qp_mode)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  if(qp_mode == Mode::Avoidance)
  {
    osqp_update_eps_rel(&workspace, 1e-6f);
  }
  else
  {
    osqp_update_eps_rel(&workspace, 1e-6f);
  }
  osqp_update_eps_abs(&workspace, 1e-4f);
  osqp_update_alpha(&workspace, 1.6);
  osqp_update_max_iter(&workspace, 50000);
  osqp_solve(&workspace);
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "opt time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  // printf("Status:                %s\n", (&workspace)->info->status);
  // printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  // printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
}

void EBPathSmoother::updateQPConstrain(
  const std::vector<geometry_msgs::Point>& interpolated_points,
  const int farrest_point_idx,
  const int num_fixed_points,
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
    else if( i < num_fixed_points)
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    }
    else
    {
      if(qp_optimization_mode == Mode::Avoidance)
      {
        double min_constrain_buffer = loose_constrain_disntance_;
        if( i >= num_fixed_points+5 && i < num_fixed_points+15)
        {
          min_constrain_buffer = 0.5;
        }
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
                    min_constrain_buffer);
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
    else if( i < num_fixed_points)
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
    }
    else
    {
      if(qp_optimization_mode == Mode::Avoidance)
      {
        double min_constrain_buffer = loose_constrain_disntance_;
        if( i >= num_fixed_points+5 && i < num_fixed_points+15)
        {
          min_constrain_buffer = 0.5;
        }
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
                    min_constrain_buffer);
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

void EBPathSmoother::updateQPConstrain(
  const std::vector<geometry_msgs::Point>& interpolated_points,
  const int num_fixed_points,
  const int farrest_point_idx,
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
    else if( i < num_fixed_points)
    {
      lower_bound[i] = interpolated_points[i].x;
      upper_bound[i] = interpolated_points[i].x;
    }
    else if( i >= num_fixed_points+5 && i < num_fixed_points+10)
    {
      lower_bound[i] = interpolated_points[i].x-1.0;
      upper_bound[i] = interpolated_points[i].x+1.0;
      geometry_msgs::Point tmp_p = interpolated_points[i];
      tmp_p.z = 1.0;
      debug_constrain_points.push_back(tmp_p);
    }
    else
    {
      lower_bound[i] = interpolated_points[i].x - 0.2;
      upper_bound[i] = interpolated_points[i].x + 0.2;
      geometry_msgs::Point tmp_p = interpolated_points[i];
      tmp_p.z = 0.2;
      debug_constrain_points.push_back(tmp_p);
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
    else if( i < num_fixed_points)
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
      upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y;
    }
    else if( i >= num_fixed_points+5 && i < num_fixed_points+10)
    // else if( i >= num_fixed_points && i < num_fixed_points+5)
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y-1.0;
      upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y+1.0;
    }
    else
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_points[i].y - 0.2;
      upper_bound[i+number_of_sampling_points_] = interpolated_points[i].y + 0.2;
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
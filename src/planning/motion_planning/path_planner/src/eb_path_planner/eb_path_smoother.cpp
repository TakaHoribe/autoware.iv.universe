
#include <vector>
#include <chrono>
#include <algorithm>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <autoware_planning_msgs/PathPoint.h>
#include <autoware_planning_msgs/TrajectoryPoint.h>

#include <ros/console.h>
#include <tf2/utils.h>

#include "workspace.h"
#include "osqp.h"
#include "eb_path_planner/horibe_interpolate.h"
#include "eb_path_planner/eb_path_smoother.h"

EBPathSmoother::EBPathSmoother(
  double exploring_minimum_radius,
  double backward_distance,
  double fixing_distance,
  double sampling_resolution):
number_of_sampling_points_(500),
exploring_minimum_radius_(exploring_minimum_radius),
backward_distance_(backward_distance),
fixing_distance_(fixing_distance),
sampling_resolution_(sampling_resolution)
{}

EBPathSmoother::~EBPathSmoother(){}

bool EBPathSmoother::preprocessExploredPoints(
    const std::vector<geometry_msgs::Point>& current_explored_points,
    const geometry_msgs::Pose& ego_pose,
    std::unique_ptr<std::vector<geometry_msgs::Point>>& previous_explored_points_ptr,
    std::unique_ptr<std::vector<double>>& previous_interpolated_x_ptr,
    std::unique_ptr<std::vector<double>>& previous_interpolated_y_ptr,
    std::vector<double>& interpolated_x,
    std::vector<double>& interpolated_y,
    int& nearest_idx,
    int& farrest_idx,
    int& nearest_idx_in_previous_optimized_points,
    std::vector<geometry_msgs::Point>& debug_interpolated_points)
{
   
  if(current_explored_points.empty() || 
     current_explored_points.size()==1)
  {
    return false;
  }
  
  std::vector<geometry_msgs::Point> modified_explored_points = current_explored_points;
  if(previous_explored_points_ptr)
  {
    bool flag = false;
    for(const auto& point: *previous_explored_points_ptr)
    {
      double dx = point.x - current_explored_points.front().x;
      double dy = point.y - current_explored_points.front().y;
      double dist = std::sqrt(dx*dx+dy*dy);
      if(dist<1e-5)
      {
        flag = true;
        break;
      }
    }
    if(!flag)
    {
      ROS_WARN("[EBPathPlanner] Coulf not find a match with previous points");
    }
    else
    {
      double min_dist1 = 99999999;
      int nearest_explored_point_idx = 0;
      for (size_t i = 0; i < current_explored_points.size(); i++)
      { 
        double dx = current_explored_points[i].x - ego_pose.position.x;
        double dy = current_explored_points[i].y - ego_pose.position.y;
        double dist = std::sqrt(dx*dx+dy*dy);
        if(dist<min_dist1)
        {
          min_dist1 = dist;
          nearest_explored_point_idx = i;
        }
      }
      modified_explored_points.clear();
      for (size_t i = nearest_explored_point_idx; i < current_explored_points.size(); i++)
      {
        modified_explored_points.push_back(current_explored_points[i]);
      }
    }
  }
  
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(size_t i = 0; i <  modified_explored_points.size(); i++)
  {
    //backward check
    if(i > 1)
    {
      double dx1 = modified_explored_points[i].x - 
                   modified_explored_points[i-1].x;
      double dy1 = modified_explored_points[i].y - 
                   modified_explored_points[i-1].y;
      double dx2 = modified_explored_points[i-1].x - 
                   modified_explored_points[i-2].x;
      double dy2 = modified_explored_points[i-1].y - 
                   modified_explored_points[i-2].y;
      double inner_product = dx1*dx2 + dy1*dy2;
      if(inner_product < 0)
      {
        std::cout << "Path points might go backwrd"  << std::endl;
      }
    }
    tmp_x.push_back(modified_explored_points[i].x);
    tmp_y.push_back(modified_explored_points[i].y);
  }
  
  if(tmp_x.empty()||tmp_y.empty())
  {
    return false;
  }
  std::vector<double> base_s = horibe_spline::calcEuclidDist(tmp_x, tmp_y);
  if(base_s.empty())
  {
    return false;
  }
  std::vector<double> new_s;
  for(double i = 0.0; 
      i <= base_s.back();
      i += sampling_resolution_)
  {
    new_s.push_back(i);
  }
  horibe_spline::SplineInterpolate spline;
  spline.interpolate(base_s, tmp_x, new_s, interpolated_x);
  spline.interpolate(base_s, tmp_y, new_s, interpolated_y);
  
  if(previous_interpolated_x_ptr &&
     previous_interpolated_y_ptr)
  {
    double nearest_interpolated_dist = 9999999;
    int nearest_previous_interpolated_point_idx_from_explored_points;
    for (size_t i = 0; i < previous_interpolated_x_ptr->size(); i++)
    {
      double dx = previous_interpolated_x_ptr->at(i) - 
                  modified_explored_points.front().x;
      double dy = previous_interpolated_y_ptr->at(i) - 
                  modified_explored_points.front().y;
      double dist = std::sqrt(dx*dx+dy*dy);
      if(dist < nearest_interpolated_dist)
      {
        nearest_interpolated_dist = dist;
        nearest_previous_interpolated_point_idx_from_explored_points = i;
      }
    }
    
    std::vector<double> previous_x;
    std::vector<double> previous_y;
    for (size_t i = 0; 
          i < nearest_previous_interpolated_point_idx_from_explored_points; i++)
    {
      previous_x.push_back(previous_interpolated_x_ptr->at(i));
      previous_y.push_back(previous_interpolated_y_ptr->at(i));
    }
    for (size_t i = 0; i < interpolated_x.size(); i++)
    {
      previous_x.push_back(interpolated_x[i]);
      previous_y.push_back(interpolated_y[i]);
    }
    interpolated_x = previous_x;
    interpolated_y = previous_y;
  }
  
  double min_dist = 100000000;
  for (int i = 0; i < interpolated_x.size(); i++)
  {
    geometry_msgs::Point tmp;
    tmp.x = interpolated_x[i];
    tmp.y = interpolated_y[i];
    tmp.z = 0.0;
    debug_interpolated_points.push_back(tmp);
    double dx = tmp.x - ego_pose.position.x;
    double dy = tmp.y - ego_pose.position.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if(dist < min_dist)
    {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  
  int num_backing_points = std::ceil(backward_distance_/sampling_resolution_);
  nearest_idx = std::max(0, nearest_idx - num_backing_points);
  farrest_idx = interpolated_x.size()-1;
  
  std::vector<double> cropped_interpolated_x;
  std::vector<double> cropped_interpolated_y;
  for (size_t i = nearest_idx; i < interpolated_x.size(); i++)
  {
    cropped_interpolated_x.push_back(interpolated_x[i]);
    cropped_interpolated_y.push_back(interpolated_y[i]);
  }
  nearest_idx_in_previous_optimized_points = nearest_idx;
  nearest_idx = 0;
  interpolated_x = cropped_interpolated_x;
  interpolated_y = cropped_interpolated_y;
  
  
  previous_explored_points_ptr=
    std::make_unique<std::vector<geometry_msgs::Point>>(current_explored_points);
  return true;
}
  
  

bool EBPathSmoother::generateOptimizedPath(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
    const std::vector<geometry_msgs::Point>& explored_points,
    const geometry_msgs::Pose& ego_pose,
    std::vector<geometry_msgs::Point>& debug_interpolated_points,
    std::vector<geometry_msgs::Point>& debug_cached_explored_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  std::vector<double> interpolated_x;
  std::vector<double> interpolated_y;
  int nearest_idx_from_ego_pose = 0;
  int farrest_idx_from_ego_pose =-1;
  int nearest_previous_interpolated_point_idx = 0;
  int nearest_idx_in_previous_optimized_points = 0;
  
  bool is_match_with_previous_points;
  bool is_preprocess_success = 
    preprocessExploredPoints(explored_points, 
                           ego_pose,
                           previous_explored_points_ptr_,
                           previous_interpolated_x_ptr_,
                           previous_interpolated_y_ptr_,
                           interpolated_x,
                           interpolated_y,
                           nearest_idx_from_ego_pose,
                           farrest_idx_from_ego_pose,
                           nearest_idx_in_previous_optimized_points,
                           debug_interpolated_points);
  if(!is_preprocess_success)
  {
    ROS_WARN("[EBPathPlanner] Preprocess for smoother return false. Skip optimization");
    return false;
  }           
  
  if(farrest_idx_from_ego_pose==nearest_idx_from_ego_pose)
  {
    ROS_WARN("[EBPathPlanner] Nearest and farrest idx is the same. Ego Pose: %lf, %lf, %lf, %lf, %lf, %lf, %lf ", 
          ego_pose.position.x, ego_pose.position.y, ego_pose.position.z, 
          ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z, ego_pose.orientation.w);
    return false;
  }
  if(farrest_idx_from_ego_pose==-1)
  {
     ROS_WARN("[EBPathPlanner] Farrest idx is -1. Ego Pose: %lf, %lf, %lf, %lf, %lf, %lf, %lf ", 
          ego_pose.position.x, ego_pose.position.y, ego_pose.position.z, 
          ego_pose.orientation.x, ego_pose.orientation.y, ego_pose.orientation.z, ego_pose.orientation.w);
     return false;
  }
  
  int fixing_idx_from_ego_pose;
  if(previous_optimized_points_ptr_)
  {
    fixing_idx_from_ego_pose = 
      std::min((int)(nearest_idx_from_ego_pose+fixing_distance_/sampling_resolution_), 
              (int)previous_optimized_points_ptr_->size());
  }
  else
  {
    fixing_idx_from_ego_pose = nearest_idx_from_ego_pose+fixing_distance_/sampling_resolution_; 
  }
   
  // int fixing_idx_from_ego_pose = 
  //   std::min((int)(nearest_idx_from_ego_pose+fixing_distance_/sampling_resolution_), 
  //            (int)previous_optimized_points_ptr_->size());
  double lower_bound[number_of_sampling_points_ * 2];
  double upper_bound[number_of_sampling_points_ * 2];
  double first_yaw = tf2::getYaw(ego_pose.orientation);
  double last_yaw = std::atan2(interpolated_y[number_of_sampling_points_-1]-
                               interpolated_y[number_of_sampling_points_-2], 
                               interpolated_x[number_of_sampling_points_-1]-
                               interpolated_x[number_of_sampling_points_-2]);
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if(nearest_idx_in_previous_optimized_points+i<fixing_idx_from_ego_pose && 
            previous_optimized_points_ptr_)
    {
      lower_bound[i] = previous_optimized_points_ptr_->at(
        nearest_idx_in_previous_optimized_points+ i).pose.position.x;
      upper_bound[i] = previous_optimized_points_ptr_->at(
        nearest_idx_in_previous_optimized_points+ i).pose.position.x;
    }
    else if (i == 0)//initial x
    {
      lower_bound[i] = interpolated_x[nearest_idx_from_ego_pose];
      upper_bound[i] = interpolated_x[nearest_idx_from_ego_pose];
    }
    else if (i == 1)//second initial x
    {
      // lower_bound[i] = new_x[nearest_idx_from_ego_pose] + 0.2 * std::cos(first_yaw);
      // upper_bound[i] = new_x[nearest_idx_from_ego_pose] + 0.2 * std::cos(first_yaw);
      lower_bound[i] = interpolated_x[nearest_idx_from_ego_pose+i];
      upper_bound[i] = interpolated_x[nearest_idx_from_ego_pose+i];
    }
    else if (i == farrest_idx_from_ego_pose - 2)//second last x
    {
      // lower_bound[i] = new_x[i+1] - 0.2 * std::cos(last_yaw);
      // upper_bound[i] = new_x[i+1] - 0.2 * std::cos(last_yaw);
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if (i == farrest_idx_from_ego_pose - 1)//last x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if(i >= farrest_idx_from_ego_pose|| 
            nearest_idx_from_ego_pose+i>=farrest_idx_from_ego_pose)
    {
      lower_bound[i] = interpolated_x[farrest_idx_from_ego_pose-1];
      upper_bound[i] = interpolated_x[farrest_idx_from_ego_pose-1];
    }
    else
    {
      lower_bound[i] = interpolated_x[nearest_idx_from_ego_pose+ i] - 0.5;
      upper_bound[i] = interpolated_x[nearest_idx_from_ego_pose+ i] + 0.5;
    }
  }
  
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if(nearest_idx_in_previous_optimized_points+i<fixing_idx_from_ego_pose && 
            previous_optimized_points_ptr_)
    {
      lower_bound[i+number_of_sampling_points_] = previous_optimized_points_ptr_->at(
        nearest_idx_in_previous_optimized_points+ i).pose.position.y;
      upper_bound[i+number_of_sampling_points_] = previous_optimized_points_ptr_->at(
        nearest_idx_in_previous_optimized_points+ i).pose.position.y;
    }
    else if (i == 0)//initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[nearest_idx_from_ego_pose];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[nearest_idx_from_ego_pose];
    }
    else if (i == 1)//second initial x
    {
      // lower_bound[i+number_of_sampling_points_] = new_y[nearest_idx_from_ego_pose] + 0.2 * std::sin(first_yaw);
      // upper_bound[i+number_of_sampling_points_] = new_y[nearest_idx_from_ego_pose] + 0.2 * std::sin(first_yaw);
      lower_bound[i+number_of_sampling_points_] = interpolated_y[nearest_idx_from_ego_pose+i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[nearest_idx_from_ego_pose+i];
    }
    else if (i == farrest_idx_from_ego_pose - 2)//second last x
    {
      // lower_bound[i+number_of_sampling_points_] = new_y[i+1] - 0.2 * std::sin(last_yaw);
      // upper_bound[i+number_of_sampling_points_] = new_y[i+1] - 0.2 * std::sin(last_yaw);
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == farrest_idx_from_ego_pose - 1)//last x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if(i >= farrest_idx_from_ego_pose|| 
            nearest_idx_from_ego_pose+i>=farrest_idx_from_ego_pose)
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[farrest_idx_from_ego_pose-1];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[farrest_idx_from_ego_pose-1];
    }
    else
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[nearest_idx_from_ego_pose+ i] - 0.5;
      upper_bound[i+number_of_sampling_points_] = interpolated_y[nearest_idx_from_ego_pose+ i] + 0.5;
    }
  }
  
  
  std::chrono::high_resolution_clock::time_point begin4 = std::chrono::high_resolution_clock::now();
  
  c_int a = osqp_update_bounds(&workspace, lower_bound, upper_bound);
  c_int b = osqp_solve(&workspace);
  std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time4 = std::chrono::duration_cast<std::chrono::nanoseconds>(end4 - begin4);
  // std::cout << "e-osqp solve " << elapsed_time4.count() / (1000.0 * 1000.0) << " milli sec" << std::endl;

  // printf("Status:                %s\n", (&workspace)->info->status);
  // printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  // printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
  int number_of_optimized_points = std::min(farrest_idx_from_ego_pose, number_of_sampling_points_);
  std::vector<geometry_msgs::Point> previous_points;
  size_t previously_used_index = 0;
  for (int i = 0; i < number_of_optimized_points; i++)
  {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = workspace.solution->x[i];
    tmp_point.pose.position.y = workspace.solution->x[i + number_of_sampling_points_];
    tmp_point.pose.position.z = ego_pose.position.z;
    bool flag = false;
    for (size_t j = previously_used_index; j < path_points.size(); j++)
    {
      double dx1 = tmp_point.pose.position.x - path_points[j].pose.position.x;
      double dy1 = tmp_point.pose.position.y - path_points[j].pose.position.y;
      double yaw = tf2::getYaw(path_points[j].pose.orientation);
      double dx2 = std::cos(yaw);
      double dy2 = std::sin(yaw);
      double inner_product = dx1*dx2+dy1*dy2;
      if(inner_product < 0)
      {
        tmp_point.twist = path_points[j].twist;
        tmp_point.pose.position.z = path_points[j].pose.position.z;
        previously_used_index = j;
        flag = true;
        break;
      }
    }
    if(!flag)
    {
      ROS_WARN("[EBPathPlanner] Could find corresponding velocity in path points. Insert 0 velocity");
    }
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    if(i==number_of_optimized_points-1)
    {
      double dx = workspace.solution->x[i] - 
                  workspace.solution->x[i-1]; 
      double dy = workspace.solution->x[i+number_of_sampling_points_] - 
                  workspace.solution->x[i-1+number_of_sampling_points_]; 
      yaw = std::atan2(dy, dx);
    }
    else
    {
      double dx = workspace.solution->x[i+1] - 
                  workspace.solution->x[i]; 
      double dy = workspace.solution->x[i+1+number_of_sampling_points_] - 
                  workspace.solution->x[i+number_of_sampling_points_]; 
      yaw = std::atan2(dy, dx);
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    tmp_point.pose.orientation = tf2::toMsg(quaternion);
    optimized_points.push_back(tmp_point);
  }
  double dx = explored_points.back().x - path_points.back().pose.position.x;
  double dy = explored_points.back().y - path_points.back().pose.position.y;
  double dist = std::sqrt(dx*dx+dy*dy);
  double diff_dist_for_goal_thres = 10;
  if(dist < diff_dist_for_goal_thres)
  {
    optimized_points.back().twist.linear.x = 0;
  }
  
  
  previous_interpolated_x_ptr_ = 
    std::make_unique<std::vector<double>>(interpolated_x);
  previous_interpolated_y_ptr_ = 
    std::make_unique<std::vector<double>>(interpolated_y);
  previous_optimized_points_ptr_= 
    std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>(optimized_points);
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "   optimization time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
} 

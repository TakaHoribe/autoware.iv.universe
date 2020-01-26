
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
#include "eb_path_planner/horibe_interpolate.h"
#include "eb_path_planner/eb_path_smoother.h"

namespace tmp
{
  
// ref: http://www.mech.tohoku-gakuin.ac.jp/rde/contents/course/robotics/coordtrans.html
// (pu, pv): retative, (px, py): absolute, (ox, oy): origin
// (pu, pv) = rot^-1 * {(px, py) - (ox, oy)}
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
                         const geometry_msgs::Pose& map_ego_pose,
                         const int ego_costmap_x_length,
                         const int ego_costmap_y_width,
                         const double costmap_resolution,
                         geometry_msgs::Point& image_point)
{
  geometry_msgs::Point relative_p = 
    transformToRelativeCoordinate2D(map_point, map_ego_pose);
  double bottomleft_x = (relative_p.x+ego_costmap_x_length/2)/costmap_resolution;
  double bottomleft_y = (relative_p.y+ego_costmap_y_width/2)/costmap_resolution;
  double image_x = ego_costmap_y_width/costmap_resolution - bottomleft_y;
  double image_y = ego_costmap_x_length/costmap_resolution - bottomleft_x;
  if(image_x>=0 && 
     image_x<(int)ego_costmap_y_width/costmap_resolution &&
     image_y>=0 && 
     image_y<(int)ego_costmap_x_length/costmap_resolution)
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



}


EBPathSmoother::EBPathSmoother(
  int number_of_fixing_points,
  double exploring_minimum_radius,
  double backward_distance,
  double fixing_distance,
  double delta_arc_length):
number_of_sampling_points_(100),
number_of_fixing_points_(number_of_fixing_points),
number_of_diff_optimization_points_for_cold_start_(40),
exploring_minimum_radius_(exploring_minimum_radius),
backward_distance_(backward_distance),
fixing_distance_(fixing_distance),
delta_arc_length_(delta_arc_length),
constrain_buffer_(0.4)
{
  cold_start(&workspace);
}

EBPathSmoother::~EBPathSmoother(){}
  
bool EBPathSmoother::preprocessExploredPoints(
    const std::vector<geometry_msgs::Point>& explored_points,
    const geometry_msgs::Pose& ego_pose,
    std::vector<double>& interpolated_x,
    std::vector<double>& interpolated_y,
    int& nearest_idx,
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
    //backward check
    if(i > 1)
    {
      double dx1 = explored_points[i].x - 
                   explored_points[i-1].x;
      double dy1 = explored_points[i].y - 
                   explored_points[i-1].y;
      double dx2 = explored_points[i-1].x - 
                   explored_points[i-2].x;
      double dy2 = explored_points[i-1].y - 
                   explored_points[i-2].y;
      double inner_product = dx1*dx2 + dy1*dy2;
      if(inner_product < 0)
      {
        std::cout << "Path points might go backwrd"  << std::endl;
        // ROS_ERROR("path mo go backward");
      }
    }
    tmp_x.push_back(explored_points[i].x);
    tmp_y.push_back(explored_points[i].y);
  }
  
  if(tmp_x.empty()||tmp_y.empty())
  {
    return false;
  }
  std::vector<double> base_s = horibe_spline::calcEuclidDist(tmp_x, tmp_y);
  if(base_s.empty())
  {
    ROS_WARN_THROTTLE(5.0, "[EBPathPlanner] Fail to calculate arc length for explored points; Skip optimization");
    return false;
  }
  std::vector<double> new_s;
  for(double i = 0.0; 
      i <= base_s.back();
      i += delta_arc_length_)
  {
    new_s.push_back(i);
  }
  horibe_spline::SplineInterpolate spline;
  spline.interpolate(base_s, tmp_x, new_s, interpolated_x);
  spline.interpolate(base_s, tmp_y, new_s, interpolated_y);
  
  for (int i = 0; i < interpolated_x.size(); i++)
  {
    geometry_msgs::Point point;
    point.x = interpolated_x[i];
    point.y = interpolated_y[i];
    debug_interpolated_points.push_back(point);
  }
  
  nearest_idx = 0;
  farrest_idx = std::min((int)(number_of_sampling_points_-1),
                         (int)(interpolated_x.size()-1));
  return true;
}
  
  

bool EBPathSmoother::generateOptimizedPath(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
    const std::vector<geometry_msgs::Point>& explored_points,
    const geometry_msgs::Pose& start_exploring_pose,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& fixed_optimized_points,
    const geometry_msgs::Pose& ego_pose,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info,
    std::vector<geometry_msgs::Point>& debug_interpolated_points,
    std::vector<geometry_msgs::Point>& debug_cached_explored_points,
    std::vector<geometry_msgs::Point>& debug_boundary_points,
    std::vector<geometry_msgs::Point>& debug_lb_boundary_points,
    std::vector<geometry_msgs::Point>& debug_ub_boundary_points,
    std::vector<geometry_msgs::Point>& debug_fixed_optimization_points,
    std::vector<geometry_msgs::Point>& debug_variable_optimization_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point begin3 = 
    std::chrono::high_resolution_clock::now();
  std::vector<double> interpolated_x;
  std::vector<double> interpolated_y;
  int nearest_idx_from_ego_pose = 0;
  int farrest_idx_from_ego_pose =-1;
  int nearest_previous_interpolated_point_idx = 0;
  int nearest_idx_in_previous_optimized_points = 0;
  
  //TODO: i dont think it needs interpolation here
  bool is_match_with_previous_points;
  bool is_preprocess_success = 
    preprocessExploredPoints(explored_points, 
                             ego_pose,
                             interpolated_x,
                             interpolated_y,
                             nearest_idx_from_ego_pose,
                             farrest_idx_from_ego_pose,
                             debug_interpolated_points);
  if(!is_preprocess_success)
  {
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
  
  // if(fixed_optimized_points.size()>=number_of_fixing_points_)
  // {
  //   std::cout << "first p "<< fixed_optimized_points[fixed_optimized_points.size()-number_of_fixing_points_].pose.position.x<< std::endl;
  //   std::cout << "first p "<< fixed_optimized_points[fixed_optimized_points.size()-number_of_fixing_points_].pose.position.y<< std::endl;
  //   std::cout << "interpo first "<< interpolated_x[0] << std::endl;
  //   std::cout << "interpo first "<< interpolated_y[0] << std::endl;
  // }
  double clearance_map_resolution = 0.1;
  int x_length = 100;
  int y_width = 100;
  double lower_bound[number_of_sampling_points_ * 2];
  double upper_bound[number_of_sampling_points_ * 2];
  double first_yaw = tf2::getYaw(start_exploring_pose.orientation);
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if(i < number_of_fixing_points_&&fixed_optimized_points.size()>number_of_fixing_points_)
    {
      lower_bound[i] = fixed_optimized_points[fixed_optimized_points.size()-number_of_fixing_points_+i].pose.position.x;
      upper_bound[i] = fixed_optimized_points[fixed_optimized_points.size()-number_of_fixing_points_+i].pose.position.x;
    }
    else if(i==0)
    // if (i == 0)//initial x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i] = interpolated_x[i-1] + delta_arc_length_ * std::cos(first_yaw);
      upper_bound[i] = interpolated_x[i-1] + delta_arc_length_ * std::cos(first_yaw);
      // lower_bound[i] = interpolated_x[nearest_idx_from_ego_pose+i];
      // upper_bound[i] = interpolated_x[nearest_idx_from_ego_pose+i];
    }
    else if (i == farrest_idx_from_ego_pose - 1)//second last x
    {
      // lower_bound[i] = new_x[i+1] - 0.2 * std::cos(last_yaw);
      // upper_bound[i] = new_x[i+1] - 0.2 * std::cos(last_yaw);
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if (i == farrest_idx_from_ego_pose )//last x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if(i >= farrest_idx_from_ego_pose)
    {
      lower_bound[i] = interpolated_x[farrest_idx_from_ego_pose];
      upper_bound[i] = interpolated_x[farrest_idx_from_ego_pose];
    }
    else
    {     
      // geometry_msgs::Point lb_x_point;
      // lb_x_point.x = interpolated_x[nearest_idx_from_ego_pose+i];
      // lb_x_point.y = interpolated_y[nearest_idx_from_ego_pose+i];
      // for (int j = 0; j < 60; j++)
      // {
      //   lb_x_point.x = lb_x_point.x - clearance_map_resolution ; 
      //   geometry_msgs::Point tmp_image_p;
      //   // if(tmp::transformMapToImage(lb_x_point, 
      //   //                     ego_pose,
      //   //                     x_length,
      //   //                     y_width,
      //   //                     clearance_map_resolution,
      //   //                     tmp_image_p))
      //   if(tmp::transformMapToImage(
      //                       lb_x_point, 
      //                       map_info,
      //                       tmp_image_p))
      //   {
      //     float clearance = 
      //       clearance_map.ptr<float>((int)tmp_image_p.y)
      //                               [(int)tmp_image_p.x];
      //     if(clearance*clearance_map_resolution <= 
      //          exploring_minimum_radius_-constrain_buffer_)
      //     {
      //       break;
      //     }
      //   }
      //   else
      //   {
      //     break;
      //   }
      // }
      // geometry_msgs::Point ub_x_point;
      // ub_x_point.x = interpolated_x[nearest_idx_from_ego_pose+i];
      // ub_x_point.y = interpolated_y[nearest_idx_from_ego_pose+i];
      // for (int j = 0; j < 60; j++)
      // {
      //   ub_x_point.x = ub_x_point.x + clearance_map_resolution; 
      //   geometry_msgs::Point tmp_image_p;
      //   // if(tmp::transformMapToImage(ub_x_point, 
      //   //                     ego_pose,
      //   //                     x_length,
      //   //                     y_width,
      //   //                     clearance_map_resolution,
      //   //                     tmp_image_p))
      //   if(tmp::transformMapToImage(
      //                       ub_x_point, 
      //                       map_info,
      //                       tmp_image_p))
      //   {
      //     float clearance = 
      //       clearance_map.ptr<float>((int)tmp_image_p.y)
      //                               [(int)tmp_image_p.x];
      //     if(clearance*clearance_map_resolution <= 
      //         exploring_minimum_radius_- constrain_buffer_)
      //     {
      //       break;
      //     }
      //   }
      //   else
      //   {
      //     break;
      //   }
      // }
      // if(ub_x_point.x - lb_x_point.x<0)
      // {
      //   ROS_ERROR("somethong wrong");
      // }
      // debug_lb_boundary_points.push_back(lb_x_point);
      
      // debug_ub_boundary_points.push_back(ub_x_point);
      // debug_boundary_points.push_back(lb_x_point);
      // debug_boundary_points.push_back(ub_x_point);
      // lower_bound[i] = lb_x_point.x;
      // upper_bound[i] = ub_x_point.x;
      // std::cout << "boud x "<< lb_x_point.x<<" "<< ub_x_point.x << std::endl;
      // lower_bound[i] = interpolated_x[i] - 0.5;
      // upper_bound[i] = interpolated_x[i] + 0.5;
      
      geometry_msgs::Point interpolated_p;
      interpolated_p.x = interpolated_x[nearest_idx_from_ego_pose+i];
      interpolated_p.y = interpolated_y[nearest_idx_from_ego_pose+i];
      geometry_msgs::Point interpolated_p_in_image;
      float clearance;
      if(tmp::transformMapToImage(
                            interpolated_p, 
                            map_info,
                            interpolated_p_in_image))
      {
        clearance = 
          clearance_map.ptr<float>((int)interpolated_p_in_image.y)
                                  [(int)interpolated_p_in_image.x]*clearance_map_resolution;
      }
      else
      {
        clearance = 0.5;
      }
      float diff = std::fmax(clearance - exploring_minimum_radius_ - constrain_buffer_, constrain_buffer_);
      lower_bound[i] = interpolated_x[i] - diff;
      upper_bound[i] = interpolated_x[i] + diff;
    }
  }
  
  // int constrain_count = 0;
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if(i < number_of_fixing_points_&& fixed_optimized_points.size()>number_of_fixing_points_)
    {
      lower_bound[i+number_of_sampling_points_] = fixed_optimized_points[fixed_optimized_points.size()-number_of_fixing_points_+i].pose.position.y;
      upper_bound[i+number_of_sampling_points_] = fixed_optimized_points[fixed_optimized_points.size()-number_of_fixing_points_+i].pose.position.y;
    }
    else if (i == 0)//initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i-1] + delta_arc_length_ * std::sin(first_yaw);
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i-1] + delta_arc_length_ * std::sin(first_yaw);
      // lower_bound[i+number_of_sampling_points_] = interpolated_y[nearest_idx_from_ego_pose+i];
      // upper_bound[i+number_of_sampling_points_] = interpolated_y[nearest_idx_from_ego_pose+i];
    }
    else if (i == farrest_idx_from_ego_pose - 1)//second last x
    {
      // lower_bound[i+number_of_sampling_points_] = new_y[i+1] - 0.2 * std::sin(last_yaw);
      // upper_bound[i+number_of_sampling_points_] = new_y[i+1] - 0.2 * std::sin(last_yaw);
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == farrest_idx_from_ego_pose)//last x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if(i >= farrest_idx_from_ego_pose)
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[farrest_idx_from_ego_pose];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[farrest_idx_from_ego_pose];
    }
    else
    {
      // geometry_msgs::Point lb_y_point;
      // lb_y_point.x = interpolated_x[nearest_idx_from_ego_pose+i];
      // lb_y_point.y = interpolated_y[nearest_idx_from_ego_pose+i];
      // for (int j = 0; j < 60; j++)
      // {
      //   lb_y_point.y = lb_y_point.y - clearance_map_resolution; 
      //   geometry_msgs::Point tmp_image_p;
      //   if(tmp::transformMapToImage(
      //                       lb_y_point, 
      //                       map_info,
      //                       tmp_image_p))
      //   {
      //     float clearance = 
      //       clearance_map.ptr<float>((int)tmp_image_p.y)
      //                               [(int)tmp_image_p.x];
      //     if(clearance*clearance_map_resolution <= 
      //          exploring_minimum_radius_- constrain_buffer_)
      //     {
      //       break;
      //     }
      //   }
      //   else
      //   {
      //     break;
      //   }
      // }
      // geometry_msgs::Point ub_y_point;
      // ub_y_point.x = interpolated_x[nearest_idx_from_ego_pose+i];
      // ub_y_point.y = interpolated_y[nearest_idx_from_ego_pose+i];
      // for (int j = 0; j < 60; j++)
      // {
      //   ub_y_point.y = ub_y_point.y + clearance_map_resolution; 
      //   geometry_msgs::Point tmp_image_p;
      //   if(tmp::transformMapToImage(
      //                       ub_y_point, 
      //                       map_info,
      //                       tmp_image_p))
      //   {
      //     float clearance = 
      //       clearance_map.ptr<float>((int)tmp_image_p.y)
      //                               [(int)tmp_image_p.x];
      //     if(clearance*clearance_map_resolution <= 
      //          exploring_minimum_radius_- constrain_buffer_)
      //     {
      //       break;
      //     }
      //   }
      //   else
      //   {
      //     break;
      //   }
      // }
      // if(ub_y_point.y - lb_y_point.y<0)
      // {
      //   ROS_ERROR("somethong wrong");
      // }
      // if(ub_y_point.y <  interpolated_y[nearest_idx_from_ego_pose+i])
      // {
      //   ROS_ERROR("somethong wrong");
      // }
      // if(lb_y_point.y >  interpolated_y[nearest_idx_from_ego_pose+i])
      // {
      //   ROS_ERROR("somethong wrong");
      // }
      // // debug_lb_boundary_points.push_back(lb_y_point);
      // // debug_ub_boundary_points.push_back(ub_y_point);
      
      // debug_lb_boundary_points[constrain_count].y = lb_y_point.y ;
      // debug_ub_boundary_points[constrain_count].y = ub_y_point.y;
      // lower_bound[i+number_of_sampling_points_] = lb_y_point.y;
      // upper_bound[i+number_of_sampling_points_] = ub_y_point.y;
      // debug_boundary_points.push_back(lb_y_point);
      // debug_boundary_points.push_back(ub_y_point);
      // lower_bound[i+number_of_sampling_points_] = interpolated_y[i] - 0.5;
      // upper_bound[i+number_of_sampling_points_] = interpolated_y[i] + 0.5;
      // constrain_count++;
      
      geometry_msgs::Point interpolated_p;
      interpolated_p.x = interpolated_x[nearest_idx_from_ego_pose+i];
      interpolated_p.y = interpolated_y[nearest_idx_from_ego_pose+i];
      geometry_msgs::Point interpolated_p_in_image;
      float clearance;
      if(tmp::transformMapToImage(
                            interpolated_p, 
                            map_info,
                            interpolated_p_in_image))
      {
        clearance = 
          clearance_map.ptr<float>((int)interpolated_p_in_image.y)
                                  [(int)interpolated_p_in_image.x]*clearance_map_resolution;
      }
      else
      {
        clearance = 0.5;
      }
      float diff = std::fmax(clearance - exploring_minimum_radius_ - constrain_buffer_, constrain_buffer_);
      // std::cout << "diff "<< diff << std::endl;
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i] - diff;
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i] + diff;
    }
  }
  
  std::chrono::high_resolution_clock::time_point end3 =
   std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time3 = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - begin3);
  std::cout << "  preprocess for optimization  "<< elapsed_time3.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  osqp_update_eps_abs(&workspace, 1e-1f);
  osqp_update_eps_rel(&workspace, 1e-5f);
  osqp_update_alpha(&workspace, 1.6);
  osqp_update_max_iter(&workspace, 16000);
  // std::cout << "polish "<< workspace.settings-> << std::endl;
  std::chrono::high_resolution_clock::time_point begin4 = std::chrono::high_resolution_clock::now();
  int number_of_optimized_points = 
    std::min(farrest_idx_from_ego_pose, number_of_sampling_points_);
  if(previous_number_of_optimized_points_ptr_)
  {
    int diff_num = number_of_optimized_points - *previous_number_of_optimized_points_ptr_;
    if(diff_num > number_of_diff_optimization_points_for_cold_start_)
    {
      cold_start(&workspace);
    }
  }
  c_int a = osqp_update_bounds(&workspace, lower_bound, upper_bound);
  c_int b = osqp_solve(&workspace);
  std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time4 = std::chrono::duration_cast<std::chrono::nanoseconds>(end4 - begin4);
  // std::cout << "e-osqp solve " << elapsed_time4.count() / (1000.0 * 1000.0) << " milli sec" << std::endl;
  printf("Status:                %s\n", (&workspace)->info->status);
  printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
  
  
  std::cout << "number of optimized pints "<< number_of_optimized_points << std::endl;
  int loop_start_ind;
  if(fixed_optimized_points.size()>number_of_fixing_points_)
  {
    loop_start_ind = std::min(number_of_fixing_points_, number_of_sampling_points_);
  }
  else
  {
    loop_start_ind = 0;
  }
  std::cout << "loop start ind  "<< loop_start_ind << std::endl;
  
  // int loop_start_ind = std::min(number_of_fixing_points_, number_of_sampling_points_);
  std::vector<geometry_msgs::Point> previous_points;
  size_t previously_used_index = 0;
  
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  //skip fixed_points
  for(size_t i = loop_start_ind; i <  number_of_optimized_points; i++)
  {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = workspace.solution->x[i];
    tmp_point.pose.position.y = workspace.solution->x[i + number_of_sampling_points_];
    tmp_x.push_back(tmp_point.pose.position.x);
    tmp_y.push_back(tmp_point.pose.position.y);
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
      i += delta_arc_length_)
  {
    new_s.push_back(i);
  }
  horibe_spline::SplineInterpolate spline;
  std::vector<double> post_interpolated_x;
  std::vector<double> post_interpolated_y;
  spline.interpolate(base_s, tmp_x, new_s, post_interpolated_x);
  spline.interpolate(base_s, tmp_y, new_s, post_interpolated_y);
  
  for (int i = 0; i < post_interpolated_x.size(); i++)
  {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = post_interpolated_x[i];
    tmp_point.pose.position.y = post_interpolated_y[i];
    tmp_point.pose.position.z = ego_pose.position.z;
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    if(i==post_interpolated_x.size()-1)
    {
      double dx = post_interpolated_x[i] - 
                  post_interpolated_x[i-1]; 
      double dy = post_interpolated_y[i] - 
                  post_interpolated_y[i-1]; 
      yaw = std::atan2(dy, dx);
    }
    else
    {
      double dx = post_interpolated_x[i+1] - 
                  post_interpolated_x[i]; 
      double dy = post_interpolated_y[i+1] - 
                  post_interpolated_y[i];
      // std::cout << "dist " << std::sqrt(dx*dx+dy*dy)<< std::endl; 
      yaw = std::atan2(dy, dx); 
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    tmp_point.pose.orientation = tf2::toMsg(quaternion);
    optimized_points.push_back(tmp_point);
  }
  
  previous_number_of_optimized_points_ptr_ = 
    std::make_unique<int>(number_of_optimized_points);
  // previous_interpolated_x_ptr_ = 
  //   std::make_unique<std::vector<double>>(interpolated_x);
  // previous_interpolated_y_ptr_ = 
  //   std::make_unique<std::vector<double>>(interpolated_y);
  // previous_optimized_points_ptr_= 
  //   std::make_unique<std::vector<autoware_planning_msgs::TrajectoryPoint>>(optimized_points);
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "  optimization time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
} 


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
    tmp_x.push_back(explored_points[i].x);
    tmp_y.push_back(explored_points[i].y);
  }
  
  std::vector<double> base_s = horibe_spline::calcEuclidDist(tmp_x, tmp_y);
  std::vector<double> new_s;
  for(double i = 0; 
      i <= base_s.back();
      i += delta_arc_length_for_explored_points_)
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
  
bool EBPathSmoother::preprocessPathPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const geometry_msgs::Pose& ego_pose,
    std::vector<double>& interpolated_x,
    std::vector<double>& interpolated_y,
    int& nearest_idx,
    int& farrest_idx,
    std::vector<geometry_msgs::Point>& debug_interpolated_points)
{
  
  
  if(path_points.empty() || 
     path_points.size()==1)
  {
    ROS_WARN("[EBPathPlanner] Almost no path points");
    return false;
  }
  
  geometry_msgs::Pose start_pose;
  double min_dist = 99999999;
  int min_ind = -1;
  int back_ind = -1;
  if(!path_points.empty())
  {
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
    back_ind = 
      std::max((int)(min_ind - 
                backward_fixing_distance_/delta_arc_length_for_path_smoothing_), 0);
    if(min_ind != -1)
    {
      start_pose = path_points[back_ind].pose;
    }
  }
  else
  {
    ROS_WARN("Path is empty");
  }
  
  
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(size_t i = back_ind;
             i <  path_points.size(); i++)
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
        std::cout << "Path points might go backwrd"  << std::endl;
      }
    }
    tmp_x.push_back(path_points[i].pose.position.x);
    tmp_y.push_back(path_points[i].pose.position.y);
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
  for(double i = 0; 
      i <= base_s.back();
      i += delta_arc_length_for_path_smoothing_)
  {
    new_s.push_back(i);
  }
  new_s.push_back(base_s.back());
  
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

bool EBPathSmoother::preprocessPathPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const geometry_msgs::Point& start_point,
    const geometry_msgs::Point& goal_point,
    const geometry_msgs::Pose& ego_pose,
    std::vector<double>& interpolated_x,
    std::vector<double>& interpolated_y,
    int& nearest_idx,
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
        std::cout << "Path points might go backwrd"  << std::endl;
      }
    }
    tmp_x.push_back(path_points[i].pose.position.x);
    tmp_y.push_back(path_points[i].pose.position.y);
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
  for(double i = 0; 
      i <= base_s.back();
      i += delta_arc_length_for_path_smoothing_)
  {
    new_s.push_back(i);
  }
  new_s.push_back(base_s.back());
  
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
  
  double clearance_map_resolution = 0.1;
  int x_length = 100;
  int y_width = 100;
  double lower_bound[number_of_sampling_points_ * 2];
  double upper_bound[number_of_sampling_points_ * 2];
  
  // geometry_msgs::Pose nearest_path_pose_from_first_point;
  // geometry_msgs::Pose nearest_path_pose_from_last_point;
  // double first_point_min_dist = 9999999;
  // double last_point_min_dist = 9999999;
  // for(const auto& point: path_points)
  // {
  //   double dx1 = point.pose.position.x - interpolated_x.front();
  //   double dy1 = point.pose.position.y - interpolated_y.front();
  //   double dist1 = std::sqrt(dx1*dx1+dy1*dy1);
  //   if(dist1 < first_point_min_dist)
  //   {
  //     first_point_min_dist = dist1;
  //     nearest_path_pose_from_first_point = point.pose;
  //   }
  //   double dx2 = point.pose.position.x - interpolated_x[farrest_idx_from_ego_pose];
  //   double dy2 = point.pose.position.y - interpolated_y[farrest_idx_from_ego_pose];
  //   double dist2 = std::sqrt(dx2*dx2+dy2*dy2);
  //   if(dist2 < last_point_min_dist)
  //   {
  //     last_point_min_dist = dist2;
  //     nearest_path_pose_from_last_point = point.pose;
  //   }
  // }
  // double first_yaw = tf2::getYaw(nearest_path_pose_from_first_point.orientation);
  // double last_yaw = tf2::getYaw(nearest_path_pose_from_last_point.orientation);
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if(i==0)
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
      // lower_bound[i] = interpolated_x[i-1]+std::cos(first_yaw);
      // upper_bound[i] = interpolated_x[i-1]+std::cos(first_yaw);
    }
    else if (i == farrest_idx_from_ego_pose - 1 )//second last x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
      // lower_bound[i] = interpolated_x[i+1]-std::cos(last_yaw);
      // upper_bound[i] = interpolated_x[i+1]-std::cos(last_yaw);
    }
    else if (i == farrest_idx_from_ego_pose )//last x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if(i > farrest_idx_from_ego_pose)
    {
      lower_bound[i] = interpolated_x[farrest_idx_from_ego_pose];
      upper_bound[i] = interpolated_x[farrest_idx_from_ego_pose];
    }
    else
    {     
      geometry_msgs::Point interpolated_p;
      interpolated_p.x = interpolated_x[i];
      interpolated_p.y = interpolated_y[i];
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
      float diff = 
        std::fmax(clearance - exploring_minimum_radius_ - tighten_constrain_disntance_,
                  loose_constrain_disntance_);
      lower_bound[i] = interpolated_x[i] - diff;
      upper_bound[i] = interpolated_x[i] + diff;
    }
  }
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if (i == 0)//initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
      // lower_bound[i+number_of_sampling_points_] = interpolated_y[i-1]+std::sin(first_yaw);
      // upper_bound[i+number_of_sampling_points_] = interpolated_y[i-1]+std::sin(first_yaw);
    }
    else if (i == farrest_idx_from_ego_pose - 1)//second last x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
      // lower_bound[i+number_of_sampling_points_] = interpolated_y[i+1]-std::sin(last_yaw);
      // upper_bound[i+number_of_sampling_points_] = interpolated_y[i+1]-std::sin(last_yaw);
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
      
      geometry_msgs::Point interpolated_p;
      interpolated_p.x = interpolated_x[i];
      interpolated_p.y = interpolated_y[i];
      geometry_msgs::Point interpolated_p_in_image;
      float clearance;
      if(tmp::transformMapToImage(
                            interpolated_p, 
                            map_info,
                            interpolated_p_in_image))
      {
        clearance = 
          clearance_map.ptr<float>
            ((int)interpolated_p_in_image.y)
            [(int)interpolated_p_in_image.x]*clearance_map_resolution;
      }
      else
      {
        clearance = 0.5;
      }
      float diff = 
        std::fmax(clearance - exploring_minimum_radius_ - tighten_constrain_disntance_,
                  loose_constrain_disntance_);
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i] - diff;
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i] + diff;
      
      interpolated_p.z = diff;
      debug_constrain_points.push_back(interpolated_p);
    }
  }
  
  //normalize objective variables
  // double sum_x, sum_y;
  // for (int i = 0; i < number_of_sampling_points_; i++)
  // {
  //   sum_x+= (lower_bound[i] + upper_bound[i]);
  //   sum_y+= (lower_bound[i+number_of_sampling_points_] + lower_bound[i+number_of_sampling_points_]);
  // }
  // double av_x = sum_x/(double)number_of_sampling_points_/2;
  // double av_y = sum_y/(double)number_of_sampling_points_/2;
  // for (int i = 0; i < number_of_sampling_points_; i++)
  // {
  //   lower_bound[i] -= av_x;
  //   upper_bound[i] -= av_x;
  //   lower_bound[i+number_of_sampling_points_] -= av_y;
  //   upper_bound[i+number_of_sampling_points_] -= av_y;
  // }
  
  
  std::chrono::high_resolution_clock::time_point end3 =
   std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time3 = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - begin3);
  std::cout << "  preprocess for optimization  "<< elapsed_time3.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  osqp_update_eps_abs(&workspace, 1e-2f);
  osqp_update_eps_rel(&workspace, 1e-5f);
  osqp_update_alpha(&workspace, 1.6);
  osqp_update_max_iter(&workspace, 16000);
  std::chrono::high_resolution_clock::time_point begin4 = std::chrono::high_resolution_clock::now();
  int number_of_optimized_points = 
    std::min(farrest_idx_from_ego_pose, number_of_sampling_points_);
  if(previous_number_of_optimized_points_ptr_)
  {
    int diff_num = number_of_optimized_points - *previous_number_of_optimized_points_ptr_;
    if(diff_num > number_of_diff_optimization_points_for_cold_start_)
    {
      ROS_WARN("Trigger cold start");
      cold_start(&workspace);
    }
  }
      // cold_start(&workspace);
  c_int a = osqp_update_bounds(&workspace, lower_bound, upper_bound);
  c_int b = osqp_solve(&workspace);
  std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time4 = std::chrono::duration_cast<std::chrono::nanoseconds>(end4 - begin4);
  // printf("Status:                %s\n", (&workspace)->info->status);
  // printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  // printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
  
  
  // std::cout << "number of optimized pints "<< number_of_optimized_points << std::endl;
  std::vector<geometry_msgs::Point> previous_points;
  size_t previously_used_index = 0;
  
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(size_t i = 0; i <= farrest_idx_from_ego_pose ; i++)
  {
    autoware_planning_msgs::TrajectoryPoint tmp_point;
    tmp_point.pose.position.x = 
      workspace.solution->x[i];
    tmp_point.pose.position.y = 
      workspace.solution->x[i + number_of_sampling_points_] ;
    // tmp_point.pose.position.x = 
    //   workspace.solution->x[i]+av_x;
    // tmp_point.pose.position.y = 
    //   workspace.solution->x[i + number_of_sampling_points_] + av_y;
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
      i += delta_arc_length_for_explored_points_)
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
      yaw = std::atan2(dy, dx); 
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    tmp_point.pose.orientation = tf2::toMsg(quaternion);
    optimized_points.push_back(tmp_point);
  }
  
  previous_number_of_optimized_points_ptr_ = 
    std::make_unique<int>(number_of_optimized_points);
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "  optimization time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
} 


bool EBPathSmoother::generateOptimizedPath(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
    std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points,
    std::vector<geometry_msgs::Point>& debug_constrain_points,
    std::vector<geometry_msgs::Point>& debug_interpolated_points)
{
  std::chrono::high_resolution_clock::time_point begin= 
    std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point begin3 = 
    std::chrono::high_resolution_clock::now();
  std::vector<double> interpolated_x;
  std::vector<double> interpolated_y;
  
  int nearest_idx_from_start_point;
  int farrest_idx_from_start_point;
  
  bool is_preprocess_success = 
    preprocessPathPoints(path_points, 
                        ego_pose,
                        interpolated_x,
                        interpolated_y,
                        nearest_idx_from_start_point,
                        farrest_idx_from_start_point,
                        debug_interpolated_points);
  
  if(!is_preprocess_success)
  {
    return false;
  }           
  
  std::cout << "farrest idx when path optimizaing " << farrest_idx_from_start_point<< std::endl;
  double clearance_map_resolution = 0.1;
  int x_length = 100;
  int y_width = 100;
  double lower_bound[number_of_sampling_points_ * 2];
  double upper_bound[number_of_sampling_points_ * 2];
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if(i==0)
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if (i == farrest_idx_from_start_point - 1 )//second last x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i]; 
    }
    else if (i == farrest_idx_from_start_point )//last x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if(i > farrest_idx_from_start_point)
    {
      lower_bound[i] = interpolated_x[farrest_idx_from_start_point];
      upper_bound[i] = interpolated_x[farrest_idx_from_start_point];
    }
    else
    { 
      lower_bound[i] = interpolated_x[i] - 0.2;
      upper_bound[i] = interpolated_x[i] + 0.2;
      
      geometry_msgs::Point interpolated_p;
      interpolated_p.x = interpolated_x[i];
      interpolated_p.y = interpolated_y[i];
      interpolated_p.z = 0.2;
      debug_constrain_points.push_back(interpolated_p);
    }
  }
  
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if (i == 0)//initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == farrest_idx_from_start_point - 1)//second last x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == farrest_idx_from_start_point)//last x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if(i > farrest_idx_from_start_point)
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[farrest_idx_from_start_point];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[farrest_idx_from_start_point];
    }
    else
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i] - 0.2;
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i] + 0.2;
    }
  }
  
  //normalize objective variables
  // double sum_x, sum_y;
  // for (int i = 0; i < number_of_sampling_points_; i++)
  // {
  //   sum_x+= (lower_bound[i] + upper_bound[i]);
  //   sum_y+= (lower_bound[i+number_of_sampling_points_] + lower_bound[i+number_of_sampling_points_]);
  // }
  // double av_x = sum_x/(double)number_of_sampling_points_/2;
  // double av_y = sum_y/(double)number_of_sampling_points_/2;
  // for (int i = 0; i < number_of_sampling_points_; i++)
  // {
  //   lower_bound[i] -= av_x;
  //   upper_bound[i] -= av_x;
  //   lower_bound[i+number_of_sampling_points_] -= av_y;
  //   upper_bound[i+number_of_sampling_points_] -= av_y;
  // }
  
  std::chrono::high_resolution_clock::time_point end3 =
   std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time3 = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - begin3);
  std::cout << "  preprocess for optimization  "<< elapsed_time3.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  osqp_update_eps_abs(&workspace, 1e-2f);
  osqp_update_eps_rel(&workspace, 1e-6f);
  osqp_update_alpha(&workspace, 1.6);
  osqp_update_max_iter(&workspace, 16000);
  std::chrono::high_resolution_clock::time_point begin4 = std::chrono::high_resolution_clock::now();
  int number_of_optimized_points = 
    std::min(farrest_idx_from_start_point, number_of_sampling_points_-1);
  if(previous_number_of_optimized_points_ptr_)
  {
    int diff_num = number_of_optimized_points - *previous_number_of_optimized_points_ptr_;
    if(diff_num > number_of_diff_optimization_points_for_cold_start_)
    {
      ROS_WARN("Trigger cold start");
      cold_start(&workspace);
    }
  }
  c_int a = osqp_update_bounds(&workspace, lower_bound, upper_bound);
  c_int b = osqp_solve(&workspace);
  std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time4 = std::chrono::duration_cast<std::chrono::nanoseconds>(end4 - begin4);
  // printf("Status:                %s\n", (&workspace)->info->status);
  // printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  // printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
  // std::cout << "last solution "<< workspace.solution->x[number_of_sampling_points_-1]<< " "<< workspace.solution->x[number_of_sampling_points_*2-1] << std::endl;
  
  
  std::vector<geometry_msgs::Point> previous_points;
  size_t previously_used_index = 0;
  
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  for(size_t i = 0; i <=  number_of_optimized_points; i++)
  {
    // autoware_planning_msgs::TrajectoryPoint tmp_point;
    // tmp_point.pose.position.x = 
    //   workspace.solution->x[i] + av_x;
    // tmp_point.pose.position.y = 
    //   workspace.solution->x[i + number_of_sampling_points_]+av_y;
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
      i += delta_arc_length_for_path_smoothing_)
  {
    new_s.push_back(i);
  }
  new_s.push_back(base_s.back());
  horibe_spline::SplineInterpolate spline;
  std::vector<double> post_interpolated_x;
  std::vector<double> post_interpolated_y;
  spline.interpolate(base_s, tmp_x, new_s, post_interpolated_x);
  spline.interpolate(base_s, tmp_y, new_s, post_interpolated_y);
  
  // std::cout << "last post interpo "<< post_interpolated_x.back()<< " "<< post_interpolated_y.back() << std::endl;
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
      yaw = std::atan2(dy, dx); 
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    tmp_point.pose.orientation = tf2::toMsg(quaternion);
    optimized_points.push_back(tmp_point);
  }
  
  previous_number_of_optimized_points_ptr_ = 
    std::make_unique<int>(number_of_optimized_points);
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
  std::chrono::high_resolution_clock::time_point begin3 = 
    std::chrono::high_resolution_clock::now();
  std::vector<double> interpolated_x;
  std::vector<double> interpolated_y;
  
  int nearest_idx_from_start_point;
  int farrest_idx_from_start_point;
  
  bool is_preprocess_success = 
    preprocessPathPoints(path_points, 
                        start_path_point,
                        goal_path_point,
                        ego_pose,
                        interpolated_x,
                        interpolated_y,
                        nearest_idx_from_start_point,
                        farrest_idx_from_start_point,
                        debug_interpolated_points);
  
  if(!is_preprocess_success)
  {
    return false;
  }           
  
  std::cout << "farrest idx when path optimizaing " << farrest_idx_from_start_point<< std::endl;
  double clearance_map_resolution = 0.1;
  int x_length = 100;
  int y_width = 100;
  double lower_bound[number_of_sampling_points_ * 2];
  double upper_bound[number_of_sampling_points_ * 2];
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if(i==0)
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if (i == farrest_idx_from_start_point - 1 )//second last x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i]; 
    }
    else if (i == farrest_idx_from_start_point )//last x
    {
      lower_bound[i] = interpolated_x[i];
      upper_bound[i] = interpolated_x[i];
    }
    else if(i > farrest_idx_from_start_point)
    {
      lower_bound[i] = interpolated_x[farrest_idx_from_start_point];
      upper_bound[i] = interpolated_x[farrest_idx_from_start_point];
    }
    else
    { 
      lower_bound[i] = interpolated_x[i] - 0.2;
      upper_bound[i] = interpolated_x[i] + 0.2;
      
      geometry_msgs::Point interpolated_p;
      interpolated_p.x = interpolated_x[i];
      interpolated_p.y = interpolated_y[i];
      interpolated_p.z = 0.2;
      debug_constrain_points.push_back(interpolated_p);
    }
  }
  
  for (int i = 0; i < number_of_sampling_points_ ; ++i)
  {
    if (i == 0)//initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == 1)//second initial x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == farrest_idx_from_start_point - 1)//second last x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if (i == farrest_idx_from_start_point)//last x
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i];
    }
    else if(i > farrest_idx_from_start_point)
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[farrest_idx_from_start_point];
      upper_bound[i+number_of_sampling_points_] = interpolated_y[farrest_idx_from_start_point];
    }
    else
    {
      lower_bound[i+number_of_sampling_points_] = interpolated_y[i] - 0.2;
      upper_bound[i+number_of_sampling_points_] = interpolated_y[i] + 0.2;
    }
  }
  
  std::chrono::high_resolution_clock::time_point end3 =
   std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time3 = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end3 - begin3);
  std::cout << "  preprocess for optimization  "<< elapsed_time3.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  osqp_update_eps_abs(&workspace, 1e-2f);
  osqp_update_eps_rel(&workspace, 1e-6f);
  osqp_update_alpha(&workspace, 1.6);
  osqp_update_max_iter(&workspace, 16000);
  std::chrono::high_resolution_clock::time_point begin4 = std::chrono::high_resolution_clock::now();
  int number_of_optimized_points = 
    std::min(farrest_idx_from_start_point, number_of_sampling_points_-1);
  if(previous_number_of_optimized_points_ptr_)
  {
    int diff_num = number_of_optimized_points - *previous_number_of_optimized_points_ptr_;
    if(diff_num > number_of_diff_optimization_points_for_cold_start_)
    {
      ROS_WARN("Trigger cold start");
      cold_start(&workspace);
    }
  }
  c_int a = osqp_update_bounds(&workspace, lower_bound, upper_bound);
  c_int b = osqp_solve(&workspace);
  std::chrono::high_resolution_clock::time_point end4 = std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds elapsed_time4 = std::chrono::duration_cast<std::chrono::nanoseconds>(end4 - begin4);
  // printf("Status:                %s\n", (&workspace)->info->status);
  // printf("Number of iterations:  %d\n", (int)((&workspace)->info->iter));
  // printf("Objective value:       %.4e\n", (&workspace)->info->obj_val);
  // std::cout << "last solution "<< workspace.solution->x[number_of_sampling_points_-1]<< " "<< workspace.solution->x[number_of_sampling_points_*2-1] << std::endl;
  
  
  std::vector<geometry_msgs::Point> previous_points;
  size_t previously_used_index = 0;
  
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
      i += delta_arc_length_for_path_smoothing_)
  {
    new_s.push_back(i);
  }
  new_s.push_back(base_s.back());
  horibe_spline::SplineInterpolate spline;
  std::vector<double> post_interpolated_x;
  std::vector<double> post_interpolated_y;
  spline.interpolate(base_s, tmp_x, new_s, post_interpolated_x);
  spline.interpolate(base_s, tmp_y, new_s, post_interpolated_y);
  
  // std::cout << "last post interpo "<< post_interpolated_x.back()<< " "<< post_interpolated_y.back() << std::endl;
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
      yaw = std::atan2(dy, dx); 
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    tmp_point.pose.orientation = tf2::toMsg(quaternion);
    optimized_points.push_back(tmp_point);
  }
  
  previous_number_of_optimized_points_ptr_ = 
    std::make_unique<int>(number_of_optimized_points);
  std::chrono::high_resolution_clock::time_point end= 
    std::chrono::high_resolution_clock::now();
  std::chrono::nanoseconds time = 
    std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin);
  std::cout << "  optimization time "<< time.count()/(1000.0*1000.0)<<" ms" <<std::endl;
  
} 

#ifndef EB_PATH_SMOOTHER__H
#define EB_PATH_SMOOTHER__H


namespace geometry_msgs
{ 
  ROS_DECLARE_MESSAGE(Pose);
  ROS_DECLARE_MESSAGE(Point);
}
namespace autoware_planning_msgs
{ 
  ROS_DECLARE_MESSAGE(PathPoint);
  ROS_DECLARE_MESSAGE(TrajectoryPoint);
}

class EBPathSmoother 
{
private:
  const int number_of_sampling_points_;
  const double exploring_minimum_radius_;
  const double backward_distance_;
  const double fixing_distance_;
  const double sampling_resolution_;
  std::unique_ptr<std::vector<geometry_msgs::Point>> previous_explored_points_ptr_;
  std::unique_ptr<std::vector<double>> previous_interpolated_x_ptr_;
  std::unique_ptr<std::vector<double>> previous_interpolated_y_ptr_;
  std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> previous_optimized_points_ptr_;
  
  bool preprocessExploredPoints(
    const std::vector<geometry_msgs::Point>& current_explored_points,
    const geometry_msgs::Pose& ego_pose,
    std::unique_ptr<std::vector<geometry_msgs::Point>>& previous_explored_points_ptr,
    std::unique_ptr<std::vector<double>>& previous_explored_x_ptr,
    std::unique_ptr<std::vector<double>>& previous_explored_y_ptr,
    std::vector<double>& intrepolated_x,
    std::vector<double>& intrepolated_y,
    int& nearest_idx,
    int& farrest_idx,
    int& nearest_idx_in_previous_optimized_points,
    std::vector<geometry_msgs::Point>& debug_interpolated_points);
    
public:
   EBPathSmoother(
     double exploring_minimum_raidus,
     double backward_distance,
     double fixing_distance,
     double sampling_resolution);
  ~EBPathSmoother();
  bool generateOptimizedPath(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<geometry_msgs::Point>& explored_points,
    const geometry_msgs::Pose& ego_pose,
    std::vector<geometry_msgs::Point>& debug_interpolated_points,                  
    std::vector<geometry_msgs::Point>& debug_cached_explored_points,                  
    std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points);                  
};

#endif
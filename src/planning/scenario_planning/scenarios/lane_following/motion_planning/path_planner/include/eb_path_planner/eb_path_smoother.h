#ifndef EB_PATH_SMOOTHER__H
#define EB_PATH_SMOOTHER__H

enum Mode
{
  Avoidance     = 0,
  LaneFollowing = 1,
};

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

namespace nav_msgs
{
  ROS_DECLARE_MESSAGE(MapMetaData); 
}


namespace cv
{
  class Mat;
}

class EBPathSmoother 
{
private:
  const int number_of_sampling_points_;
  const int number_of_diff_optimization_points_for_cold_start_;
  const double exploring_minimum_radius_;
  const double backward_fixing_distance_;
  const double fixing_distance_;
  const double delta_arc_length_for_path_smoothing_;
  const double delta_arc_length_for_explored_points_;
  const double loose_constrain_disntance_;
  const double tighten_constrain_disntance_;
  
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
  
  bool preprocessExploredPoints(
    const std::vector<geometry_msgs::Point>& explored_points,
    const geometry_msgs::Pose& ego_pose,
    std::vector<double>& interpolated_x,
    std::vector<double>& interpolated_y,
    std::vector<geometry_msgs::Point>& interpolated_points,
    int& farrest_idx,
    std::vector<geometry_msgs::Point>& debug_interpolated_points);
  
  bool preprocessPathPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const geometry_msgs::Point& start_point,
    const geometry_msgs::Point& goal_point,
    const geometry_msgs::Pose& ego_pose,
    std::vector<double>& interpolated_x,
    std::vector<double>& interpolated_y,
    std::vector<geometry_msgs::Point>& interpolated_points,
    int& farrest_idx,
    std::vector<geometry_msgs::Point>& debug_interpolated_points);
    
  void updateQPConstrain(
    const std::vector<geometry_msgs::Point>& interpolated_points,
    const int farrest_point_idx,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info,
    const Mode qp_optimization_mode,
    std::vector<geometry_msgs::Point>& debug_constrain_points);
    
  void solveQP(const Mode qp_optimization_mode);
  
  std::vector<autoware_planning_msgs::TrajectoryPoint> 
    generatePostProcessedTrajectoryPoints(
      const int number_of_optimized_points,
      const geometry_msgs::Pose& ego_pose);
  
public:
   EBPathSmoother(
     double exploring_minimum_raidus,
     double backward_distance,
     double fixing_distance,
     double delta_arc_length_for_path_smoothing,
     double delta_arc_length_for_explored_points);
  ~EBPathSmoother();
  
  bool generateOptimizedExploredPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<geometry_msgs::Point>& explored_points,
    const geometry_msgs::Pose& ego_pose,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info,
    std::vector<geometry_msgs::Point>& debug_interpolated_points,                  
    std::vector<geometry_msgs::Point>& debug_constrain_points,                  
    std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points);                  
  
  bool generateOptimizedPath(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
    const geometry_msgs::Point& start_path_point,
    const geometry_msgs::Point& goal_path_point,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points,
    std::vector<geometry_msgs::Point>& debug_constrain_points,
    std::vector<geometry_msgs::Point>& debug_interpolated_points);
};

#endif
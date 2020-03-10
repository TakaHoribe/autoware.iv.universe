#ifndef EB_PATH_SMOOTHER__H
#define EB_PATH_SMOOTHER__H

#include <Eigen/Core>

enum Mode : int {
  Avoidance = 0,
  LaneFollowing = 1,
};

namespace geometry_msgs {
ROS_DECLARE_MESSAGE(Pose);
ROS_DECLARE_MESSAGE(Point);
}  // namespace geometry_msgs
namespace autoware_planning_msgs {
ROS_DECLARE_MESSAGE(PathPoint);
ROS_DECLARE_MESSAGE(TrajectoryPoint);
}  // namespace autoware_planning_msgs

namespace nav_msgs {
ROS_DECLARE_MESSAGE(MapMetaData);
}

namespace cv {
class Mat;
}

namespace osqp {
class OSQPInterface;
}

class EBPathSmoother {
 private:
  const int number_of_sampling_points_;
  const int number_of_diff_optimization_points_for_cold_start_;
  const double exploring_minimum_radius_;
  const double backward_fixing_distance_;
  const double fixing_distance_;
  const double delta_arc_length_for_path_smoothing_;
  const double delta_arc_length_for_explored_points_;
  const double loose_constrain_disntance_;
  std::unique_ptr<osqp::OSQPInterface> osqp_solver_ptr_;

  void initializeSolver();

  Eigen::MatrixXd makePMatrix();

  void updateQPConstrain(const std::vector<geometry_msgs::Point>& interpolated_points, const int farrest_point_idx,
                         const int num_fixed_points, const cv::Mat& clearance_map,
                         const nav_msgs::MapMetaData& map_info, const Mode qp_optimization_mode,
                         std::vector<geometry_msgs::Point>& debug_constrain_points);

  void updateQPConstrain(const std::vector<geometry_msgs::Point>& interpolated_points, const int num_fixed_points,
                         const int farrest_point_idx, std::vector<geometry_msgs::Point>& debug_constrain);

  std::vector<double> solveQP();

 public:
  EBPathSmoother(double exploring_minimum_raidus, double backward_distance, double fixing_distance,
                 double delta_arc_length_for_path_smoothing, double delta_arc_length_for_explored_points);
  ~EBPathSmoother();

  std::vector<autoware_planning_msgs::TrajectoryPoint> generateOptimizedExploredPoints(
      const std::vector<autoware_planning_msgs::PathPoint>& path_points,
      const std::vector<geometry_msgs::Point>& fixed_points, const std::vector<geometry_msgs::Point>& non_fixed_points,
      const geometry_msgs::Pose& ego_pose, const cv::Mat& clearance_map, const nav_msgs::MapMetaData& map_info,
      std::vector<geometry_msgs::Point>& debug_interpolated_points,
      std::vector<geometry_msgs::Point>& debug_constrain_points);

  bool generateOptimizedPath(const geometry_msgs::Pose& ego_pose,
                             const std::vector<autoware_planning_msgs::PathPoint>& path_points,
                             const geometry_msgs::Point& start_path_point, const geometry_msgs::Point& goal_path_point,
                             std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points,
                             std::vector<geometry_msgs::Point>& debug_constrain_points,
                             std::vector<geometry_msgs::Point>& debug_interpolated_points);

  std::vector<autoware_planning_msgs::TrajectoryPoint> generateOptimizedPoints(
      const std::vector<autoware_planning_msgs::PathPoint>& path_points,
      const std::vector<geometry_msgs::Pose>& fixed_points, const std::vector<geometry_msgs::Pose>& non_fixed_points,
      std::vector<geometry_msgs::Point>& debug_inter, std::vector<geometry_msgs::Point>& debug_constrain);
};

#endif
#ifndef EB_PATH_PLANNER_H
#define EB_PATH_PLANNER_H
#include "path_base_planner/node.hpp"


namespace cv
{
  class Mat;
}

namespace std_msgs
{
  ROS_DECLARE_MESSAGE(Bool); 
}


namespace autoware_planning_msgs
{
  ROS_DECLARE_MESSAGE(Path);
  ROS_DECLARE_MESSAGE(TrajectoryPoint);
  
}

namespace autoware_perception_msgs
{
  ROS_DECLARE_MESSAGE(DynamicObjectArray);
  ROS_DECLARE_MESSAGE(DynamicObject);
}

namespace nav_msgs
{
  ROS_DECLARE_MESSAGE(OccupancyGrid);
}

namespace geometry_msgs
{ 
  ROS_DECLARE_MESSAGE(PoseStamped);
  ROS_DECLARE_MESSAGE(PoseWithCovarianceStamped);
  ROS_DECLARE_MESSAGE(Pose);
  ROS_DECLARE_MESSAGE(TwistStamped);
}

class ModifyReferencePath;
class EBPathSmoother;

namespace path_planner
{
class EBPathPlannerNode : public  BasePlannerNode
{
private:
  bool is_debug_clearance_map_mode_;
  bool is_debug_drivable_area_mode_;
  bool is_publishing_clearance_map_as_occupancy_grid_;
  bool is_previously_avoidance_mode_;
  bool enable_avoidance_;
  int number_of_backward_path_points_for_detecting_objects_;
  double forward_fixing_distance_;
  double backward_fixing_distance_;
  double detecting_objects_radius_from_ego_;
  double detecting_objects_radius_around_path_point_;
  double exploring_minimum_radius_;
  double delta_arc_length_for_path_smoothing_;
  double delta_arc_length_for_explored_points_;
  double max_avoiding_objects_velocity_ms_;
  double clearance_weight_when_exploring_;
  double exploring_goal_clearance_from_obstacle_;
  double fixing_point_clearance_from_obstacle_;
  double min_distance_threshold_when_switching_avoindance_to_path_following_;
  double min_cos_similarity_when_switching_avoindance_to_path_following_;
  geometry_msgs::Pose::ConstPtr current_ego_pose_;
  std::unique_ptr<ModifyReferencePath> modify_reference_path_ptr_;
  std::unique_ptr<EBPathSmoother> eb_path_smoother_ptr_;
  std::unique_ptr<geometry_msgs::Point> previous_ego_point_ptr_;
  std::unique_ptr<geometry_msgs::Point> previous_start_path_point_ptr_;
  std::unique_ptr<geometry_msgs::Point> previous_goal_path_point_ptr_;
  // std::unique_ptr<geometry_msgs::Point> previous_goal_point_for_exploration_ptr_;
  std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> previous_optimized_points_ptr_;
  std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>> previous_path_points_ptr_;
  // std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> previous_optimized_explored_points_ptr_;
  std::shared_ptr<autoware_perception_msgs::DynamicObjectArray> in_objects_ptr_;
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher markers_pub_;
  ros::Publisher debug_clearance_map_in_occupancy_grid_pub_;
  ros::Subscriber objects_sub_;
  ros::Subscriber initial_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber enable_avoidance_sub_;
  void callback(const autoware_planning_msgs::Path &input_path_msg, 
                autoware_planning_msgs::Trajectory &output_trajectory_msg) override;
  void objectsCallback(const autoware_perception_msgs   ::DynamicObjectArray& msg);
  void initialposeCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);
  void goalCallback(const geometry_msgs::PoseStamped& msg);
  void enableAvoidanceCallback(const std_msgs::Bool& msg);
  void initializing();
  void resettingPtrForAvoidance();
  void resettingPtrForLaneFollowing();
  
  bool needResetPrevOptimizedExploredPoints(
    const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>&
      previous_optimized_explored_points_ptr,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects);
  
  bool isAvoidanceNeeded(
    const std::vector<autoware_planning_msgs::PathPoint> in_path,
    const geometry_msgs::Pose self_pose,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& previous_output_trajectory_points);
    
  bool isDetectingFixedPathPoint(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points);
    
  bool getNearestPose(
    const geometry_msgs::Pose self_pose,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& trajectory_points,
    geometry_msgs::Pose& nearest_pose);
  
  bool isPoseCloseToPath(
    const std::vector<autoware_planning_msgs::PathPoint> in_path,
    const geometry_msgs::Pose in_pose);
  
  bool needExprolation(
    const geometry_msgs::Point& ego_point,
    const autoware_planning_msgs::Path& in_path,
    const cv::Mat& clearance_map,
    const std::vector<geometry_msgs::Point>& fixed_explored_points,
    geometry_msgs::Point& start_exploring_point,
    geometry_msgs::Point& goal_exploring_point);
  
  std::vector<geometry_msgs::Point> generateFixedOptimizedExploredPoints(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<geometry_msgs::Point>& explored_points,
    const cv::Mat& clearance_map,
    const cv::Mat& only_objects_clearance_map,
    const nav_msgs::MapMetaData& map_info);
  
  std::unique_ptr<std::vector<geometry_msgs::Point>> 
    generateNonFixedExploredPoints(
      const autoware_planning_msgs::Path& input_path,
      const std::vector<geometry_msgs::Point>& fixed_explored_points,
      const cv::Mat& clearance_map);
    
  bool alighWithPathPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points);
    
  bool generateFineOptimizedTrajectory(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& merged_optimized_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& fine_optimized_points);
  
  bool drawObstalcesOnImage(
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const nav_msgs::MapMetaData& map_info,
    cv::Mat& image);
  
  bool generateClearanceMap(
    const nav_msgs::OccupancyGrid& occupancy_grid,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const geometry_msgs::Pose& debug_ego_pose,
    cv::Mat& clearance_map);
  
  cv::Mat generateOnlyObjectsClearanceMap(
    const cv::Mat& clearance_map,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const nav_msgs::MapMetaData& map_info);
    
  bool areValidStartAndGoal(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::unique_ptr<geometry_msgs::Point>& start_point,
    const std::unique_ptr<geometry_msgs::Point>& goal_point,
    const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>& prev_optimized_path);
  
  bool calculateNewStartAndGoal(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    std::unique_ptr<geometry_msgs::Point>& start_point,
    std::unique_ptr<geometry_msgs::Point>& goal_point);
  
  bool detectAvoidingObjectsOnPath(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points);
    
  bool detectAvoidingObjectsOnPoints(
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& prev_optimized_explored_points);
  
  autoware_planning_msgs::Trajectory::Ptr generateSmoothTrajectoryFromPath(
                            const autoware_planning_msgs::Path& input_path);
                            
  autoware_planning_msgs::Trajectory::Ptr generateSmoothTrajectoryFromExploredPoints(
    const autoware_planning_msgs::Path& input_path);
  
  void getOccupancyGridValue(const nav_msgs::OccupancyGrid& occupancy_grid, 
                             const int i, 
                             const int j,
                             unsigned char&value);
                             
  void putOccupancyGridValue(nav_msgs::OccupancyGrid& occupancy_grid, 
                             const int i, 
                             const int j,
                             const unsigned char&value);
                             
  bool convertPathToSmoothTrajectory(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& smoothed_points);
  
  std::unique_ptr<std::vector<geometry_msgs::Pose>> 
    generateValidFixedPathPoints(
      const std::vector<autoware_planning_msgs::PathPoint>& path_points,
      const std::vector<geometry_msgs::Pose>& fixed_path_points);
  
  std::vector<geometry_msgs::Pose> 
    generateFixedOptimizedPathPoints(
      const geometry_msgs::Pose& ego_pose,
      const std::vector<autoware_planning_msgs::PathPoint>& path_points,
      const std::unique_ptr<geometry_msgs::Point>& start_point,
      std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>& prev_optimized_path);
      
  bool 
    isPathShapeChanged(
      const geometry_msgs::Pose& ego_pose,
      const std::vector<autoware_planning_msgs::PathPoint>& path_points,
      const std::unique_ptr<std::vector<autoware_planning_msgs::PathPoint>>& prev_path_points);
  
  std::vector<geometry_msgs::Pose>
    generateNonFixedPoints(
      const std::vector<autoware_planning_msgs::PathPoint>& path_points,
      const std::vector<geometry_msgs::Pose>& fixed_points,
      const std::unique_ptr<geometry_msgs::Point>& goal_point);
    
  autoware_planning_msgs::Trajectory generateSmoothTrajectory(
    const autoware_planning_msgs::Path& in_path);
  
  
  void debugFixedNonFixedPointsMarker(
      const std::vector<geometry_msgs::Point>& fixed,
      const std::vector<geometry_msgs::Point>& non_fixed);
  
  void debugStartAndGoalMarkers(
    const geometry_msgs::Point& start_point,
    const geometry_msgs::Point& goal_point);
    
  void debugMarkers(
    const std::vector<geometry_msgs::Point>& constrain_points,
    const std::vector<geometry_msgs::Point>& interpolated_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& optimized_points);
  
public:
   EBPathPlannerNode();
  ~EBPathPlannerNode();
};
}

#endif
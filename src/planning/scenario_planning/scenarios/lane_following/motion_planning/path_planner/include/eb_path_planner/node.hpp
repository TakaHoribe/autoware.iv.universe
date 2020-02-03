#ifndef EB_PATH_PLANNER_H
#define EB_PATH_PLANNER_H
#include "path_base_planner/node.hpp"

namespace lanelet
{
  class Lanelet;
  class LaneletMap;
  using LaneletMapPtr = std::shared_ptr<LaneletMap>;
  namespace routing
  {
    class RoutingGraph;
  }
  namespace traffic_rules
  {
    class TrafficRules;
  }
}

namespace cv
{
  class Mat;
}

namespace std_msgs
{
  ROS_DECLARE_MESSAGE(Bool); 
}

namespace autoware_lanelet2_msgs
{
  ROS_DECLARE_MESSAGE(MapBin); 
}


namespace autoware_planning_msgs
{
  ROS_DECLARE_MESSAGE(Route);
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
  void callback(const autoware_planning_msgs::Path &input_path_msg, 
                autoware_planning_msgs::Trajectory &output_trajectory_msg) override;

  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher path_pub_;
  ros::Publisher debug_traj_pub_;
  ros::Publisher debug_fixed_traj_pub_;
  ros::Publisher markers_pub_;
  ros::Subscriber is_relay_path_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber objects_sub_;
  bool is_relaying_path_mode_;
  bool use_optimization_when_relaying_;
  int number_of_fixing_explored_points_;
  double backward_fixing_distance_;
  double exploring_minimum_radius_;
  double forward_fixing_distance_;
  double delta_arc_length_for_path_smoothing_;
  double delta_arc_length_for_explored_points_;
  double max_avoiding_objects_velocity_ms_;
  
  std::unique_ptr<ModifyReferencePath> modify_reference_path_ptr_;
  std::unique_ptr<EBPathSmoother> eb_path_smoother_ptr_;
  std::unique_ptr<geometry_msgs::Point> previous_ego_point_ptr_;
  std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> previous_optimized_points_ptr_;
  std::unique_ptr<std::vector<geometry_msgs::Point>> previous_explored_points_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> in_twist_ptr_;
  std::shared_ptr<autoware_perception_msgs::DynamicObjectArray> in_objects_ptr_;
  void isRelayPathCallback(const std_msgs::Bool& msg);
  void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
  void objectsCallback(const autoware_perception_msgs::DynamicObjectArray& msg);
  void doResetting();
  
  bool needReset(
    const geometry_msgs::Point& current_ego_point,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info,
    const std::vector<geometry_msgs::Point>& fixed_explored_points);
  
  bool generateFixedExploredPoints(
    const geometry_msgs::Pose& ego_pose,
    const std::unique_ptr<std::vector<geometry_msgs::Point>>& previous_explored_points_ptr,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info,
    std::vector<geometry_msgs::Point>& fixed_points);
    
  bool alighWithPathPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& merged_optimized_points);
  
  bool generateFineOptimizedPoints(
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& merged_optimized_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& fine_optimized_points);
  
  bool generateClearanceMap(
    const nav_msgs::OccupancyGrid& occupancy_grid,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const geometry_msgs::Pose& debug_ego_pose,
    cv::Mat& clearance_map);
  
  bool detectAvoidingObjectsOnPath(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points);
  
  bool generateSmoothTrajectory(const geometry_msgs::Pose& ego_pose,
                            const autoware_planning_msgs::Path& input_path,
                            autoware_planning_msgs::Trajectory& output_trajectory);
  
  void getOccupancyGridValue(const nav_msgs::OccupancyGrid& occupancy_grid, 
                             const int i, 
                             const int j,
                             unsigned char&value);
                             
  bool convertPathToSmoothTrajectory(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& smoothed_points,
    std::vector<geometry_msgs::Point>& debug_fixed_optimzied_points_used_for_constrain,
    std::vector<geometry_msgs::Point>& debug_interpolated_points_used_for_optimization);
  
public:
   EBPathPlannerNode();
  ~EBPathPlannerNode();
};
}

#endif
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
  // ros::Publisher traj_pub_;
  ros::Publisher markers_pub_;
  // ros::Subscriber bin_map_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber route_sub_;
  ros::Subscriber objects_sub_;
  ros::Subscriber avoid_mode_sub_;
  bool enable_velocity_based_cropping_;
  bool is_debug_no_fixing_points_mode_;
  bool is_relaying_path_mode_;
  bool use_optimization_when_relaying_;
  int number_of_fixing_points_;
  double time_for_calculating_velocity_based_distance_;
  double distance_for_cropping_;
  double backward_fixing_distance_;
  double exploring_minimum_radius_;
  double forward_fixing_distance_;
  double delta_arc_length_;
  double delta_ego_point_threshold_;
  double max_avoiding_objects_velocity_ms_;
  
  std::unique_ptr<ModifyReferencePath> modify_reference_path_ptr_;
  std::unique_ptr<EBPathSmoother> eb_path_smoother_ptr_;
  std::unique_ptr<geometry_msgs::Point> previous_ego_point_ptr_;
  std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>> previous_optimized_points_ptr_;
  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
  
  std::shared_ptr<autoware_planning_msgs::Route> in_route_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> in_twist_ptr_;
  std::shared_ptr<autoware_perception_msgs::DynamicObjectArray> in_objects_ptr_;
  void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
  void mapCallback(const autoware_lanelet2_msgs::MapBin& msg);
  void routeCallback(const autoware_planning_msgs::Route& msg);
  void objectsCallback(const autoware_perception_msgs::DynamicObjectArray& msg);
  bool needReset(const geometry_msgs::Point& previous_ego_point,
                 const geometry_msgs::Point& current_ego_point,
                 const cv::Mat& clearance_map,
                 const nav_msgs::MapMetaData& map_info,
                 const std::vector<autoware_planning_msgs::TrajectoryPoint>& fixed_optimized_points);
  bool generateFixedOptimizedPoints(
    const geometry_msgs::Pose& ego_pose,
    const std::unique_ptr<std::vector<autoware_planning_msgs::TrajectoryPoint>>& previous_optimized_points_ptr,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& fixed_points);
    
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
  
  void getOccupancyGridValue(const nav_msgs::OccupancyGrid& occupancy_grid, 
                             const int i, 
                             const int j,
                             unsigned char&value);
                             
  bool convertPathToSmoothTrajectory(
    const geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<autoware_planning_msgs::TrajectoryPoint>& fixed_optimized_points,
    std::vector<autoware_planning_msgs::TrajectoryPoint>& smoothed_points,
    std::vector<geometry_msgs::Point>& debug_fixed_optimzied_points_used_for_constrain,
    std::vector<geometry_msgs::Point>& debug_interpolated_points_used_for_optimization);
  
public:
   EBPathPlannerNode();
  ~EBPathPlannerNode();
};
}

#endif
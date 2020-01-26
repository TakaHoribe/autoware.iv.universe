#ifndef MODIFY_REFERENCE_PATH_H
#define MODIFY_REFERENCE_PATH_H

namespace lanelet
{
  // class Lanelet;
  class LaneletMap;
  // using LaneletMapPtr = std::shared_ptr<LaneletMap>;
  namespace routing
  {
    class RoutingGraph;
  }
}

namespace cv
{
  class Mat;
} // namespace cv

struct Node;


namespace autoware_planning_msgs
{
  ROS_DECLARE_MESSAGE(PathPoint); 
  ROS_DECLARE_MESSAGE(Route); 
}
namespace autoware_perception_msgs
{
  ROS_DECLARE_MESSAGE(DynamicObject); 
}

namespace geometry_msgs
{ 
  // ROS_DECLARE_MESSAGE(PoseStamped);
  ROS_DECLARE_MESSAGE(Pose);
  ROS_DECLARE_MESSAGE(Point);
  // ROS_DECLARE_MESSAGE(TwistStamped);
}

namespace nav_msgs
{
  ROS_DECLARE_MESSAGE(MapMetaData); 
}



class ModifyReferencePath 
{
private:
  bool is_fix_pose_mode_for_debug_;
  bool is_debug_each_iteration_mode_;
  bool is_debug_driveable_area_mode_;
  bool is_debug_clearance_map_mode_;
  int clearance_map_y_width_;
  int clearance_map_x_length_;
  //should be deprecated when implementing appropriate driveable area
  int num_lookup_lanelet_for_drivealble_area_; 
  double resolution_;
  double time_limit_;
  double min_radius_;
  double max_radius_;
  double backward_distance_;
  double static_objects_velocity_ms_threshold_;
  double loosing_clerance_for_explore_goal_threshold_;
  double heuristic_epsilon_;
  std::unique_ptr<geometry_msgs::Pose> debug_fix_pose_;
  std::unique_ptr<geometry_msgs::Pose> previous_exploring_goal_pose_in_map_ptr_;
  std::unique_ptr<std::vector<geometry_msgs::Point>> cached_explored_points_ptr_;
  bool expandNode(Node& parent_node, 
                  const cv::Mat& clearence_map,
                  const nav_msgs::MapMetaData& map_info,
                  const Node& goal_node,
                  const double min_r,
                  const double max_r,
                  std::vector<Node>& expanded_nodes);
  bool isOverlap(Node& node1, Node& node2);
  bool nodeExistInClosedNodes(Node node, std::vector<Node> closed_nodes);
  bool solveGraphAStar(const geometry_msgs::Pose& ego_pose,
                     const geometry_msgs::Point& start_point_in_map,
                     const geometry_msgs::Point& goal_point_in_map,
                     const cv::Mat& clearance_map,
                     const nav_msgs::MapMetaData& map_info,
                     std::vector<geometry_msgs::Point>& explored_points);
  
  bool arrangeExploredPointsBaseedOnClearance(
    const cv::Mat& clearance_map,
    const geometry_msgs::Pose& ego_pose,
    std::vector<geometry_msgs::Point>& explored_points,
    std::vector<geometry_msgs::Point>& debug_rearranged_points);
                        
public:
  bool generateModifiedPath(
    geometry_msgs::Pose& ego_pose,
    const geometry_msgs::Pose& start_exploring_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    const lanelet::routing::RoutingGraph& graph,
    lanelet::LaneletMap& map,
    const autoware_planning_msgs::Route& route,
    std::vector<geometry_msgs::Point>& explored_points,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info,
    geometry_msgs::Point& debug_goal_point,
    std::vector<geometry_msgs::Point>& debug_rearrange_points
  );
  ModifyReferencePath(
    int num_lookup_lanelet_for_driveable_area,
    double min_radius,
    double backward_distance);
  ModifyReferencePath();
  ~ModifyReferencePath();
};

#endif
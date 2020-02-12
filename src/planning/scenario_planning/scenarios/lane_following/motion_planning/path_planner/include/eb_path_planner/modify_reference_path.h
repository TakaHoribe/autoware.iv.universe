#ifndef MODIFY_REFERENCE_PATH_H
#define MODIFY_REFERENCE_PATH_H

// namespace lanelet
// {
//   // class Lanelet;
//   class LaneletMap;
//   // using LaneletMapPtr = std::shared_ptr<LaneletMap>;
//   namespace routing
//   {
//     class RoutingGraph;
//   }
// }

namespace cv
{
  class Mat;
} // namespace cv

class Node;
class GridNode;

namespace autoware_planning_msgs
{
  ROS_DECLARE_MESSAGE(PathPoint); 
  // ROS_DECLARE_MESSAGE(Route); 
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
  int clearance_map_y_width_;
  int clearance_map_x_length_;
  //TODO: should be deprecated when implementing appropriate driveable area
  double resolution_;
  double time_limit_millisecond_;
  double min_radius_;
  double max_radius_;
  double backward_distance_;
  double clearance_weight_when_exploring_;
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
  bool expandGridNode(
                const GridNode& parent_node, 
                const cv::Mat& clearance_map,
                const nav_msgs::MapMetaData& map_info,
                const GridNode& goal_node,
                cv::Mat& visited_map,
                std::vector<GridNode>& child_nodes);
  bool isOverlap(Node& node1, Node& node2);
  bool nodeExistInClosedNodes(Node node, std::vector<Node> closed_nodes);
  bool solveGraphAStar(const geometry_msgs::Pose& ego_pose,
                     const geometry_msgs::Point& start_point_in_map,
                     const geometry_msgs::Point& goal_point_in_map,
                     const cv::Mat& clearance_map,
                     const nav_msgs::MapMetaData& map_info,
                     std::vector<geometry_msgs::Point>& explored_points);
  bool solveAStar(
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
    const geometry_msgs::Point& start_exploring_point,
    const geometry_msgs::Point& goal_exploring_point,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const std::vector<autoware_perception_msgs::DynamicObject>& objects,
    std::vector<geometry_msgs::Point>& explored_points,
    const cv::Mat& clearance_map,
    const nav_msgs::MapMetaData& map_info);
  ModifyReferencePath(
    double min_radius,
    double backward_distance,
    double clearance_weight_when_exploring);
  ModifyReferencePath();
  ~ModifyReferencePath();
};

#endif
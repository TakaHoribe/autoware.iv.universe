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

namespace geometry_msgs
{ 
  // ROS_DECLARE_MESSAGE(PoseStamped);
  ROS_DECLARE_MESSAGE(Pose);
  ROS_DECLARE_MESSAGE(Point);
  // ROS_DECLARE_MESSAGE(TwistStamped);
}



class ModifyReferencePath 
{
private:
  bool is_fix_pose_mode_for_debug_;
  bool is_debug_graph_a_star_mode_;
  bool is_debug_each_iteration_mode_;
  bool is_debug_driveable_area_mode_;
  int y_width_;
  int x_length_;
  //should be deprecated when implementing appropriate driveable area
  int num_lookup_lanelet_for_drivealble_area_; 
  double resolution_;
  double time_limit_;
  double min_radius_;
  double max_radius_;
  double backward_distance_;
  std::unique_ptr<geometry_msgs::Pose> debug_fix_pose_;
  std::unique_ptr<geometry_msgs::Pose> previous_exploring_goal_point_in_map_ptr_;
  std::unique_ptr<std::vector<geometry_msgs::Point>> cached_explored_points_ptr_;
  bool expandNode(Node& parent_node, 
                  const cv::Mat& clearence_map,
                  const Node& goal_node,
                  const double min_r,
                  const double max_r,
                  std::vector<Node>& expanded_nodes,
                  Node& lowest_f_child_node,
                  int& lowest_f_child_node_index);
  bool isOverlap(Node& node1, Node& node2);
  bool nodeExistInClosedNodes(Node node, std::vector<Node> closed_nodes);
  bool solveGraphAStar(const geometry_msgs::Pose& ego_pose,
                     const geometry_msgs::Point& start_point_in_map,
                     const geometry_msgs::Point& goal_point_in_map,
                     const cv::Mat& clearance_map,
                     std::vector<geometry_msgs::Point>& explored_points);
  bool needExploration(
    const geometry_msgs::Pose& ego_pose,
    const geometry_msgs::Point& goal_point_in_map,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points, 
    std::unique_ptr<std::vector<geometry_msgs::Point>>& cached_explored_points_ptr,
    geometry_msgs::Point& start_point_in_map);
                        
public:
  bool generateModifiedPath(
    geometry_msgs::Pose& ego_pose,
    const std::vector<autoware_planning_msgs::PathPoint>& path_points,
    const lanelet::routing::RoutingGraph& graph,
    lanelet::LaneletMap& map,
    const autoware_planning_msgs::Route& route,
    std::vector<geometry_msgs::Point>& debug_points,
    geometry_msgs::Point& debug_goal_point
  );
   ModifyReferencePath(
     int num_lookup_lanelet_for_driveable_area,
     double min_radius,
     double backward_distance);
  ~ModifyReferencePath();
};

#endif
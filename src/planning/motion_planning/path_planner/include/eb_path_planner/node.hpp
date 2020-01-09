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

namespace autoware_lanelet2_msgs
{
  ROS_DECLARE_MESSAGE(MapBin); 
}


namespace autoware_planning_msgs
{
  ROS_DECLARE_MESSAGE(Path);
  ROS_DECLARE_MESSAGE(Route);
  
}

namespace autoware_perception_msgs
{
  ROS_DECLARE_MESSAGE(DynamicObjectArray);
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
  // ros::Publisher traj_pub_;
  ros::Publisher markers_pub_;
  // ros::Subscriber bin_map_sub_;
  ros::Subscriber twist_sub_;
  ros::Subscriber map_sub_;
  ros::Subscriber route_sub_;
  ros::Subscriber objects_sub_;
  bool enable_velocity_based_cropping_; 
  int num_lookup_lanelet_for_drivealble_area_;
  double time_for_calculating_velocity_based_distance_;
  double distance_for_cropping_;
  double backward_distance_;
  double exploring_minimum_radius_;
  double fixing_distance_;
  double sampling_resolution_;
  double delta_ego_point_threshold_;
  
  std::unique_ptr<ModifyReferencePath> modify_reference_path_ptr_;
  std::unique_ptr<EBPathSmoother> eb_path_smoother_ptr_;
  std::unique_ptr<geometry_msgs::Point> previous_ego_point_ptr_;
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
                 const geometry_msgs::Point& current_ego_point);
  
public:
   EBPathPlannerNode();
  ~EBPathPlannerNode();
};
}

#endif
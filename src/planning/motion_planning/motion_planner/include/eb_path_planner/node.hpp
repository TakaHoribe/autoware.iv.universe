#ifndef EB_PATH_PLANNER_H
#define EB_PATH_PLANNER_H
#include "motion_base_planner/node.hpp"

// namespace autoware_lanelet2_msgs
// {
//   ROS_DECLARE_MESSAGE(MapBin); 
// }
// namespace lanelet
// {
//    class LaneletMap;
//    namespace routing
//    {
//      class RoutingGraph;
//    }
// }

namespace autoware_planning_msgs
{
  ROS_DECLARE_MESSAGE(Path); 
}

namespace geometry_msgs
{ 
  ROS_DECLARE_MESSAGE(PoseStamped);
  ROS_DECLARE_MESSAGE(Pose);
  ROS_DECLARE_MESSAGE(TwistStamped);
}



namespace motion_planner
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
  bool enable_velocity_based_cropping_; 
  double time_for_calculating_velocity_based_distance_;
  double distance_for_cropping_;
  // ros::Timer timer_;
  
  // std::shared_ptr<lanelet::LaneletMap> kept_lanelet_map_;
  // std::unique_ptr<lanelet::routing::RoutingGraph> kept_map_routing_graph_;
  // std::shared_ptr<autoware_lanelet2_msgs::MapBin> in_map_ptr_;
  // void binMapCallback(const autoware_lanelet2_msgs::MapBin& msg);
  
  // std::unique_ptr<geometry_msgs::TransformStaped> gridmap2map_tf_;
  // std::unique_ptr<geometry_msgs::TransformStamped> lidar2map_tf_;
  // std::unique_ptr<geometry_msgs::TransformStamped> map2gridmap_tf_;
  
  // std::unique_ptr<geometry_msgs::Pose> ego_pose_ptr_;
  // std::unique_ptr<lanelet::LaneletMapPtr> kept_lanelet_map_ptr_;
  
  // std::shared_ptr<autoware_msgs::Lane> in_waypoints_ptr_;
  // geometry_msgs::PoseStamped in_pose_ptr_;
  // std::shared_ptr<geometry_msgs::PoseStamped> in_pose_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> in_twist_ptr_;
  // std::shared_ptr<autoware_planning_msgs::Path> in_path_ptr_;
  // std::shared_ptr<autoware_msgs::DetectedObjectArray> in_objects_ptr_;
  
  // void waypointsCallback(const autoware_msgs::Lane& msg);
  // void currentPoseCallback(const geometry_msgs::PoseStamped& msg);j
  void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
  // void objectsCallback(const autoware_msgs::DetectedObjectArray& msg);
  // void pathCallback(const autoware_planning_msgs::Path &msg);
  // void timerCallback(const ros::TimerEvent &e);
  
public:
   EBPathPlannerNode();
  ~EBPathPlannerNode();
};
}

#endif
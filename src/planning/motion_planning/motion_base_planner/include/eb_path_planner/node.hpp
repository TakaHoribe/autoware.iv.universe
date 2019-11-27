// #pragma once
// #include <autoware_planning_msgs/Path.h>
// #include <autoware_planning_msgs/Trajectory.h>
// #include <ros/ros.h>
// #include <memory>
// #include <tf2_ros/transform_listener.h>

// namespace motion_planner
// {
// class BasePlannerNode
// {
// protected:
//   ros::NodeHandle nh_, pnh_;
//   ros::Publisher path_pub_;
//   ros::Subscriber path_sub_;
//   tf2_ros::Buffer tf_buffer_;
//   tf2_ros::TransformListener tf_listener_;
//   virtual bool callback(const autoware_planning_msgs::Path &input_path_msg, autoware_planning_msgs::Path &output_path_msg) = 0;
//   void pathCallback(const autoware_planning_msgs::Path &input_path_msg);
//   bool getSelfPose(geometry_msgs::TransformStamped& self_pose, const std_msgs::Header &header);
//   bool getCurrentSelfPose(geometry_msgs::TransformStamped& self_pose);

// public:
//   BasePlannerNode();
//   virtual ~BasePlannerNode(){};
// };
// } // namespace motion_planner

#ifndef EB_PATH_PLANNER_H
#define EB_PATH_PLANNER_H
#include "motion_base_planner/node.hpp"

// namespace autoware_msgs
// {
//   ROS_DECLARE_MESSAGE(Lane); 
//   ROS_DECLARE_MESSAGE(Waypoint); 
// }

namespace geometry_msgs
{ 
  ROS_DECLARE_MESSAGE(PoseStamped);
  // ROS_DECLARE_MESSAGE(Pose);
  ROS_DECLARE_MESSAGE(TwistStamped);
  // ROS_DECLARE_MESSAGE(TransformStamped);
}

namespace motion_planner
{
class EBPathPlannerNode : public  BasePlannerNode
{
private:
  void callback(const autoware_planning_msgs::Path &input_path_msg, 
                autoware_planning_msgs::Path &output_path_msg) override;

  // ros
  ros::NodeHandle nh_, private_nh_;
  ros::Publisher final_waypoints_pub_;
  ros::Publisher markers_pub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber current_velocity_sub_;
  ros::Subscriber safety_waypoints_sub_;
  ros::Subscriber objects_sub_;
  
  
  ros::Timer timer_;
  
  // std::unique_ptr<geometry_msgs::TransformStaped> gridmap2map_tf_;
  // std::unique_ptr<geometry_msgs::TransformStamped> lidar2map_tf_;
  // std::unique_ptr<geometry_msgs::TransformStamped> map2gridmap_tf_;
  
  // std::shared_ptr<autoware_msgs::Lane> in_waypoints_ptr_;
  // geometry_msgs::PoseStamped in_pose_ptr_;
  std::shared_ptr<geometry_msgs::PoseStamped> in_pose_ptr_;
  std::shared_ptr<geometry_msgs::TwistStamped> in_twist_ptr_;
  // std::shared_ptr<autoware_msgs::DetectedObjectArray> in_objects_ptr_;
  
  // void waypointsCallback(const autoware_msgs::Lane& msg);
  // void currentPoseCallback(const geometry_msgs::PoseStamped& msg);
  // void currentVelocityCallback(const geometry_msgs::TwistStamped& msg);
  // void objectsCallback(const autoware_msgs::DetectedObjectArray& msg);
  void timerCallback(const ros::TimerEvent &e);
  
public:
   EBPathPlannerNode();
  ~EBPathPlannerNode();
};
}

#endif
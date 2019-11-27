
// #include <autoware_msgs/Lane.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include <memory>

#include "eb_path_planner/node.hpp"


namespace motion_planner
{
  
  EBPathPlannerNode::EBPathPlannerNode()
    : nh_(), 
    private_nh_("~")
  {
    // final_waypoints_pub_ = nh_.advertise<autoware_msgs::Lane>("final_waypoints", 1, true);
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("velocity_planner_debug_markes", 1, true);
    // safety_waypoints_sub_ = nh_.subscribe("safety_waypoints", 1, &EBPathPlannerNode::waypointsCallback, this);
    // current_pose_sub_ = nh_.subscribe("/current_pose", 1, &EBPathPlannerNode::currentPoseCallback, this);
    // current_velocity_sub_ = nh_.subscribe("/current_velocity", 1, &EBPathPlannerNode::currentVelocityCallback, this);
    // objects_sub_ = nh_.subscribe("/detection/lidar_detector/objects", 1, &QPPlannerROS::objectsCallback, this);
    
    // double timer_callback_dt = 0.05;
    double timer_callback_delta_second = 0.1;
    // double timer_callback_delta_second = 1.0;
    // double timer_callback_delta_second = 0.5;
    timer_ = nh_.createTimer(ros::Duration(timer_callback_delta_second), &EBPathPlannerNode::timerCallback, this);
  }
  
  EBPathPlannerNode::~EBPathPlannerNode(){}
  
  void EBPathPlannerNode::callback(const autoware_planning_msgs::Path &input_path_msg, 
                                                  autoware_planning_msgs::Path &output_path_msg)
  {
    
  }

//   // void EBPathPlannerNode::waypointsCallback(const autoware_msgs::Lane& msg)
//   // {
//   //   in_waypoints_ptr_ = std::make_shared<autoware_msgs::Lane>(msg);
//   // }

//   void EBPathPlannerNode::currentPoseCallback(const geometry_msgs::PoseStamped & msg)
//   {
//     in_pose_ptr_ = std::make_shared<geometry_msgs::PoseStamped>(msg);
//   }

//   void EBPathPlannerNode::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
//   {
//     in_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(msg); 
//   }
  
  void EBPathPlannerNode::timerCallback(const ros::TimerEvent &e)
  {
    // if(!in_pose_ptr_)
    // {
    //   std::cerr << "pose not arrive" << std::endl;
    // }
    // if(!in_twist_ptr_)
    // {
    //   std::cerr << "twist not arrive" << std::endl;
    // }
    // if(!in_waypoints_ptr_)
    // {
    //   std::cerr << "waypoints not arrive" << std::endl;
    // }
    
    // if(in_pose_ptr_ && 
    //   in_twist_ptr_ && 
    //   in_waypoints_ptr_) 
    // { 
    //   final_waypoints_pub_.publish(*in_waypoints_ptr_);
    // }
  }

}// end namespace
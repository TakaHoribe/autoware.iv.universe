
// #include <autoware_msgs/Lane.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_lanelet2_msgs/MapBin.h>


// #include <lanelet2_projection/UTM.h>
// #include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/Forward.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>

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
    bin_map_sub_ = nh_.subscribe("/lanelet_map_bin", 1, &EBPathPlannerNode::binMapCallback,this);
    
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
    std::cerr << "aaaaa"  << std::endl;
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
  
  void EBPathPlannerNode::binMapCallback(const autoware_lanelet2_msgs::MapBin& msg)
  {
    std::cerr << "map"  << std::endl;
    lanelet::LaneletMapPtr viz_lanelet_map(new lanelet::LaneletMap);

    lanelet::utils::conversion::fromBinMsg(msg, viz_lanelet_map);
    ROS_INFO("Map is loaded\n");

    // get lanelets etc to visualize
    lanelet::ConstLanelets all_lanelets = lanelet::utils::query::laneletLayer(viz_lanelet_map);
    lanelet::ConstLanelets road_lanelets = lanelet::utils::query::roadLanelets(all_lanelets);
    kept_road_lanelets_.reset(new lanelet::ConstLanelets(road_lanelets));
    // double lss = 0.2;  // line string size
    // visualization_msgs::MarkerArray marker_array;
    // for (auto li = lanelets.begin(); li != lanelets.end(); li++)
    // {
    //   lanelet::ConstLanelet lll = *li;

    //   lanelet::ConstLineString3d left_ls = lll.leftBound();
    //   lanelet::ConstLineString3d right_ls = lll.rightBound();
    //   lanelet::ConstLineString3d center_ls = lll.centerline();

    //   visualization_msgs::Marker left_line_strip, right_line_strip, center_line_strip;

    //   visualization::lineString2Marker(left_ls, &left_line_strip, "map", "left_lane_bound", c, lss);
    //   visualization::lineString2Marker(right_ls, &right_line_strip, "map", "right_lane_bound", c, lss);
    //   marker_array.markers.push_back(left_line_strip);
    //   marker_array.markers.push_back(right_line_strip);
    //   if (viz_centerline)
    //   {
    //     visualization::lineString2Marker(center_ls, &center_line_strip, "map", "center_lane_line", c, lss * 0.5);
    //     marker_array.markers.push_back(center_line_strip);
    //   }
    // }
  }

}// end namespace
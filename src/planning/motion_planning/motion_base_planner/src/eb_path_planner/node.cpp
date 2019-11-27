
// #include <autoware_msgs/Lane.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_lanelet2_msgs/MapBin.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_routing/LaneletPath.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>
#include <lanelet2_extension/utility/message_conversion.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_extension/visualization/visualization.h>
#include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>

#include <tf2/utils.h>

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
    double timer_callback_delta_second = 1.0;
    // double timer_callback_delta_second = 1.0;
    // double timer_callback_delta_second = 0.5;
    timer_ = nh_.createTimer(ros::Duration(timer_callback_delta_second), &EBPathPlannerNode::timerCallback, this);
  }
  
  EBPathPlannerNode::~EBPathPlannerNode(){}
  
  void EBPathPlannerNode::callback(const autoware_planning_msgs::Path &input_path_msg, 
                                                  autoware_planning_msgs::Path &output_path_msg)
  {
    
  }
  
  void EBPathPlannerNode::timerCallback(const ros::TimerEvent &e)
  {
    std::cerr << "aaaaa"  << std::endl;
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(
        "map",
        "base_link",
        ros::Time(0));
      // tf_buffer_
      std::cerr << "position in map " << transform.transform.translation.x<<" "<<
                                         transform.transform.translation.y<<" "<<
                                         transform.transform.translation.z<<std::endl;
      geometry_msgs::Pose pose;
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation.x = transform.transform.rotation.x;
      pose.orientation.y = transform.transform.rotation.y;
      pose.orientation.z = transform.transform.rotation.z;
      pose.orientation.w = transform.transform.rotation.z;
      ego_pose_ptr_.reset(new geometry_msgs::Pose(pose));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
    }
    
    if(kept_lanelet_map_&&kept_map_routing_graph_)
    {
      std::vector<std::pair<double, lanelet::Lanelet>> closest_lanelets =
      lanelet::geometry::findNearest(kept_lanelet_map_->laneletLayer, 
                                     lanelet::BasicPoint2d(ego_pose_ptr_->position.x,
                                                           ego_pose_ptr_->position.y),
                                     1);
      lanelet::Lanelet closest_lanelet = closest_lanelets.front().second;
      
      lanelet::routing::LaneletPaths paths =  
        kept_map_routing_graph_->possiblePaths(closest_lanelet, 
                                                100, 0,true);
      std::vector<lanelet::Point3d> connected_centerline;
      std::cerr << "path size " << paths.size() << std::endl;
      // connected_centerline.front().x;
      for(const auto& path: paths)
      {
        for(const auto& lanelet: path)
        {
          std::cerr << "points per lalet " << lanelet.centerline().size() << std::endl;
          for(const auto& pt: lanelet.centerline())
          {
            // std::cerr << "pt " << pt.x() << std::endl;
          }
        }
        std::cerr << "------"  << std::endl;
      }
    }
  }
  
  void EBPathPlannerNode::binMapCallback(const autoware_lanelet2_msgs::MapBin& msg)
  {
    if(!kept_lanelet_map_|| !kept_map_routing_graph_)
    {
      kept_lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
      std::cerr << "map"  << std::endl;
      lanelet::utils::conversion::fromBinMsg(msg, kept_lanelet_map_);
      lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
        lanelet::traffic_rules::TrafficRulesFactory::create(
          lanelet::Locations::Germany, 
          lanelet::Participants::Vehicle);
      kept_map_routing_graph_ =
        lanelet::routing::RoutingGraph::build(*kept_lanelet_map_, *traffic_rules);
      std::cerr << "finish"  << std::endl;
    }
  }
}// end namespace

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
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_planner_debug_markes", 1, true);
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
        "map", //target
        "base_link", //src
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
      return;
    }
    
    std::vector<std::vector<geometry_msgs::Point>> geometry_paths;
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
        std::vector<geometry_msgs::Point> geometry_path;
        for(const auto& lanelet: path)
        {
          geometry_msgs::Point geometry_point;
          std::cerr << "points per lalet " << lanelet.centerline().size() << std::endl;
          for(const auto& pt: lanelet.centerline())
          {
            geometry_point.x = pt.x();
            geometry_point.y = pt.y();
            geometry_point.z = pt.z();
            std::cerr << "pt " << pt.x() << std::endl;
            geometry_path.push_back(geometry_point);
          }
          geometry_paths.push_back(geometry_path);
        }
        std::cerr << "------"  << std::endl;
      }
    }
    
    //debug; marker array
    visualization_msgs::MarkerArray marker_array;
    int unique_id = 0;
    
    // visualize gridmap point
    visualization_msgs::Marker debug_path_planner_marker;
    debug_path_planner_marker.lifetime = ros::Duration(0.2);
    debug_path_planner_marker.header = transform.header;
    debug_path_planner_marker.ns = std::string("debug_path_planner_marker");
    debug_path_planner_marker.action = visualization_msgs::Marker::MODIFY;
    debug_path_planner_marker.pose.orientation.w = 1.0;
    debug_path_planner_marker.id = unique_id;
    debug_path_planner_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    debug_path_planner_marker.scale.x = 1.0f;
    debug_path_planner_marker.scale.y = 0.1f;
    debug_path_planner_marker.scale.z = 0.1f;
    debug_path_planner_marker.color.r = 1.0f;
    debug_path_planner_marker.color.g = 1.0f;
    debug_path_planner_marker.color.a = 1;
    for(const auto& path: geometry_paths)
    {
      for(const auto& point: path)
      {
        debug_path_planner_marker.points.push_back(point);
      }
    }
    
    marker_array.markers.push_back(debug_path_planner_marker);
    unique_id++;
    markers_pub_.publish(marker_array);
    
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
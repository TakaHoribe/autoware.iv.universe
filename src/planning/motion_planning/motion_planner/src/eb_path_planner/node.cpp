
// #include <autoware_msgs/Lane.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>

// #include <autoware_lanelet2_msgs/MapBin.h>
// #include <lanelet2_core/LaneletMap.h>
// #include <lanelet2_core/geometry/Lanelet.h>
// #include <lanelet2_routing/LaneletPath.h>
// #include <lanelet2_routing/RoutingGraph.h>
// #include <lanelet2_traffic_rules/TrafficRulesFactory.h>
// #include <lanelet2_extension/utility/message_conversion.h>
// #include <lanelet2_extension/utility/query.h>
// #include <lanelet2_extension/visualization/visualization.h>
// #include <lanelet2_extension/regulatory_elements/autoware_traffic_light.h>

#include <tf2/utils.h>

#include <memory>

#include "eb_path_planner/reference_path.hpp"

#include "eb_path_planner/node.hpp"


namespace motion_planner
{
  
  EBPathPlannerNode::EBPathPlannerNode()
    : nh_(), 
    private_nh_("~")
  {
    // double timer_callback_dt = 0.05;   
    double timer_callback_delta_second = 1.0;
    // double timer_callback_delta_second = 1.0;
    // double timer_callback_delta_second = 0.5;
    timer_ = nh_.createTimer(ros::Duration(timer_callback_delta_second), &EBPathPlannerNode::timerCallback, this);
    
    markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_planner_debug_markes", 1, true);
    path_pub_ = nh_.advertise<autoware_planning_msgs::Path>("planning/motion_planning/avoiding_path", 1, true);
    traj_pub_ = nh_.advertise<autoware_planning_msgs::Trajectory>("planning/motion_planning/tmp_trajectory_from_path_planner", 1, true);
    // bin_map_sub_ = nh_.subscribe("/lanelet_map_bin", 1, &EBPathPlannerNode::binMapCallback,this);
    path_sub_ = nh_.subscribe("/path", 1, &EBPathPlannerNode::pathCallback,this);
    
    // private_nh_.param<double>("constant_velocity", use_height_, false);
    
  }
  
  EBPathPlannerNode::~EBPathPlannerNode(){}
  
  void EBPathPlannerNode::callback(const autoware_planning_msgs::Path &input_path_msg, 
                                                  autoware_planning_msgs::Path &output_path_msg)
  {
    
  }
  
  void EBPathPlannerNode::timerCallback(const ros::TimerEvent &e)
  {
    geometry_msgs::TransformStamped transform;
    try
    {
      transform = tf_buffer_.lookupTransform(
        "map", //target
        "base_link", //src
        ros::Time(0));
      geometry_msgs::Pose pose;
      pose.position.x = transform.transform.translation.x;
      pose.position.y = transform.transform.translation.y;
      pose.position.z = transform.transform.translation.z;
      pose.orientation.x = transform.transform.rotation.x;
      pose.orientation.y = transform.transform.rotation.y;
      pose.orientation.z = transform.transform.rotation.z;
      pose.orientation.w = transform.transform.rotation.w;
      ego_pose_ptr_.reset(new geometry_msgs::Pose(pose));
    }
    catch (tf2::TransformException& ex)
    {
      ROS_WARN("%s", ex.what());
      return;
    }
    
    if(in_path_ptr_)
    {
      double min_dist = 999999;
      size_t min_index = 0;
      for(size_t i= 0; i< in_path_ptr_->points.size(); i++)
      {
        double dx = ego_pose_ptr_->position.x - in_path_ptr_->points[i].pose.position.x;
        double dy = ego_pose_ptr_->position.y - in_path_ptr_->points[i].pose.position.y;
        double dist = std::sqrt(std::pow(dx, 2)+std::pow(dy,2));
        if(dist < min_dist)
        {
          min_dist = dist;
          min_index = i;
        }
      }
      std::vector<double> tmp_x;
      std::vector<double> tmp_y;
      std::vector<double> tmp_v;
      for(size_t i=min_index; i< min_index+50; i++)
      {
        tmp_x.push_back(in_path_ptr_->points[i].pose.position.x);
        tmp_y.push_back(in_path_ptr_->points[i].pose.position.y);
        tmp_v.push_back(in_path_ptr_->points[i].twist.linear.x);
      }
      // ReferencePath reference_path(tmp_x, tmp_y, 0.1);
      ReferenceTrajectoryPath reference_trajectory_path(tmp_x, tmp_y, tmp_v,0.1);
      
      autoware_planning_msgs::Path path_msg;
      autoware_planning_msgs::Trajectory traj_msg;
      path_msg.header = transform.header;
      traj_msg.header = transform.header;
      for(size_t i = 0; i < reference_trajectory_path.x_.size(); i++)
      {
        autoware_planning_msgs::PathPoint path_point_msg;
        path_point_msg.pose.position.x = reference_trajectory_path.x_[i];
        path_point_msg.pose.position.y = reference_trajectory_path.y_[i];
        path_point_msg.pose.position.z = ego_pose_ptr_->position.z;
        path_point_msg.pose.orientation = ego_pose_ptr_->orientation;
        path_point_msg.twist.linear.x = reference_trajectory_path.v_[i];
        path_msg.points.push_back(path_point_msg);
        
        autoware_planning_msgs::TrajectoryPoint traj_point_msg;
        traj_point_msg.pose = path_point_msg.pose;
        traj_point_msg.twist = path_point_msg.twist;
        traj_msg.points.push_back(traj_point_msg);
      }
      path_pub_.publish(path_msg);
      traj_pub_.publish(traj_msg);
      
      
      
      //debug; marker array
      visualization_msgs::MarkerArray marker_array;
      int unique_id = 0;

      // visualize cubic spline point
      visualization_msgs::Marker debug_cubic_spline;
      debug_cubic_spline.lifetime = ros::Duration(1.0);
      debug_cubic_spline.header = transform.header;
      debug_cubic_spline.ns = std::string("debug_cubic_spline");
      debug_cubic_spline.action = visualization_msgs::Marker::MODIFY;
      debug_cubic_spline.pose.orientation.w = 1.0;
      debug_cubic_spline.id = unique_id;
      debug_cubic_spline.type = visualization_msgs::Marker::SPHERE_LIST;
      debug_cubic_spline.scale.x = 1.0f;
      debug_cubic_spline.scale.y = 0.1f;
      debug_cubic_spline.scale.z = 0.1f;
      debug_cubic_spline.color.g = 1.0f;
      debug_cubic_spline.color.a = 1;
      for(size_t i = 0; i< reference_trajectory_path.x_.size(); i++)
      {
        geometry_msgs::Point point;
        point.x = reference_trajectory_path.x_[i];
        point.y = reference_trajectory_path.y_[i];
        debug_cubic_spline.points.push_back(point);
      }
      
      marker_array.markers.push_back(debug_cubic_spline);
      unique_id++;
      
      markers_pub_.publish(marker_array);
    }
    
    
    // std::vector<std::vector<geometry_msgs::Point>> geometry_paths;
    // if(kept_lanelet_map_&&kept_map_routing_graph_)
    // {
    //   std::vector<std::pair<double, lanelet::Lanelet>> closest_lanelets =
    //   lanelet::geometry::findNearest(kept_lanelet_map_->laneletLayer, 
    //                                  lanelet::BasicPoint2d(ego_pose_ptr_->position.x,
    //                                                        ego_pose_ptr_->position.y),
    //                                  1);
    //   lanelet::Lanelet closest_lanelet = closest_lanelets.front().second;
      
    //   lanelet::routing::LaneletPaths paths =  
    //     kept_map_routing_graph_->possiblePaths(closest_lanelet, 
    //                                             100, 0,true);
    //   for(const auto& path: paths)
    //   {
    //     std::vector<geometry_msgs::Point> geometry_path;
    //     for(const auto& lanelet: path)
    //     {
    //       geometry_msgs::Point geometry_point;
    //       for(const auto& pt: lanelet.centerline())
    //       {
    //         geometry_point.x = pt.x();
    //         geometry_point.y = pt.y();
    //         geometry_point.z = pt.z();
    //         geometry_path.push_back(geometry_point);
    //       }
    //       geometry_paths.push_back(geometry_path);
    //     }
    //   }
    // }
   
    // //interpolate with spline
    // std::vector<double> tmp_x;
    // std::vector<double> tmp_y;
    // if(geometry_paths.size()==0)
    // {
    //   std::cerr << "there is no lanelet"  << std::endl;
    //   return;
    // }
    // for (const auto& point: geometry_paths.front())
    // {
    //   tmp_x.push_back(point.x);
    //   tmp_y.push_back(point.y);
    // }
    // ReferencePath reference_path(tmp_x, tmp_y, 0.1);
    
    // autoware_planning_msgs::Path path_msg;
    // autoware_planning_msgs::Trajectory traj_msg;
    // path_msg.header = transform.header;
    // traj_msg.header = transform.header;
    // for(size_t i = 0; i < reference_path.x_.size(); i++)
    // {
    //   autoware_planning_msgs::PathPoint path_point_msg;
    //   path_point_msg.pose.position.x = reference_path.x_[i];
    //   path_point_msg.pose.position.y = reference_path.y_[i];
    //   path_point_msg.pose.position.z = ego_pose_ptr_->position.z;
    //   path_point_msg.pose.orientation = ego_pose_ptr_->orientation;
    //   path_point_msg.twist.linear.x = 1.38;
    //   path_msg.points.push_back(path_point_msg);
      
    //   autoware_planning_msgs::TrajectoryPoint traj_point_msg;
    //   traj_point_msg.pose = path_point_msg.pose;
    //   traj_point_msg.twist = path_point_msg.twist;
    //   traj_msg.points.push_back(traj_point_msg);
    // }
    // path_pub_.publish(path_msg);
    // traj_pub_.publish(traj_msg);
    
    // //debug; marker array
    // visualization_msgs::MarkerArray marker_array;
    // int unique_id = 0;
    
    // // visualize gridmap point
    // visualization_msgs::Marker debug_path_planner_marker;
    // debug_path_planner_marker.lifetime = ros::Duration(1.0);
    // debug_path_planner_marker.header = transform.header;
    // debug_path_planner_marker.ns = std::string("debug_path_planner_marker");
    // debug_path_planner_marker.action = visualization_msgs::Marker::MODIFY;
    // debug_path_planner_marker.pose.orientation.w = 1.0;
    // debug_path_planner_marker.id = unique_id;
    // debug_path_planner_marker.type = visualization_msgs::Marker::SPHERE_LIST;
    // debug_path_planner_marker.scale.x = 1.0f;
    // debug_path_planner_marker.scale.y = 0.1f;
    // debug_path_planner_marker.scale.z = 0.1f;
    // debug_path_planner_marker.color.r = 1.0f;
    // debug_path_planner_marker.color.g = 1.0f;
    // debug_path_planner_marker.color.a = 1;
    // for(const auto& path: geometry_paths)
    // {
    //   for(const auto& point: path)
    //   {
    //     debug_path_planner_marker.points.push_back(point);
    //   }
    // }
    
    // marker_array.markers.push_back(debug_path_planner_marker);
    // unique_id++;
    
    // // visualize cubic spline point
    // visualization_msgs::Marker debug_cubic_spline;
    // debug_cubic_spline.lifetime = ros::Duration(1.0);
    // debug_cubic_spline.header = transform.header;
    // debug_cubic_spline.ns = std::string("debug_cubic_spline");
    // debug_cubic_spline.action = visualization_msgs::Marker::MODIFY;
    // debug_cubic_spline.pose.orientation.w = 1.0;
    // debug_cubic_spline.id = unique_id;
    // debug_cubic_spline.type = visualization_msgs::Marker::SPHERE_LIST;
    // debug_cubic_spline.scale.x = 1.0f;
    // debug_cubic_spline.scale.y = 0.1f;
    // debug_cubic_spline.scale.z = 0.1f;
    // debug_cubic_spline.color.g = 1.0f;
    // debug_cubic_spline.color.a = 1;
    // for(size_t i = 0; i< reference_path.x_.size(); i++)
    // {
    //   geometry_msgs::Point point;
    //   point.x = reference_path.x_[i];
    //   point.y = reference_path.y_[i];
    //   debug_cubic_spline.points.push_back(point);
    // }
    
    // marker_array.markers.push_back(debug_cubic_spline);
    // unique_id++;
    
    // markers_pub_.publish(marker_array);
    
  }
  
  void EBPathPlannerNode::pathCallback(const autoware_planning_msgs::Path &msg)
  {
    in_path_ptr_ = std::make_shared<autoware_planning_msgs::Path>(msg);
  }
  // void EBPathPlannerNode::binMapCallback(const autoware_lanelet2_msgs::MapBin& msg)
  // {
  //   if(!kept_lanelet_map_ || !kept_map_routing_graph_)
  //   {
  //     std::cerr << "start loading lalet"  << std::endl;
  //     kept_lanelet_map_ = std::make_shared<lanelet::LaneletMap>();
  //     lanelet::utils::conversion::fromBinMsg(msg, kept_lanelet_map_);
  //     lanelet::traffic_rules::TrafficRulesPtr traffic_rules =
  //       lanelet::traffic_rules::TrafficRulesFactory::create(
  //         lanelet::Locations::Germany, 
  //         lanelet::Participants::Vehicle);
  //     kept_map_routing_graph_ =
  //       lanelet::routing::RoutingGraph::build(*kept_lanelet_map_, *traffic_rules);
  //     std::cerr << "finished loading lanelet"  << std::endl;
  //   }
  // }
  
}// end namespace
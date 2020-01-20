#pragma once

#include <memory>

#include <ros/ros.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <autoware_lanelet2_msgs/MapBin.h>
#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Scenario.h>
#include <autoware_planning_msgs/Trajectory.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_traffic_rules/TrafficRules.h>

struct Input {
  ros::Subscriber sub_trajectory;

  autoware_planning_msgs::Trajectory::ConstPtr buf_trajectory;
};

struct Output {
  ros::Publisher pub_scenario;
  ros::Publisher pub_trajectory;
};

class ScenarioSelectorNode {
 public:
  ScenarioSelectorNode();

  void onMap(const autoware_lanelet2_msgs::MapBin& msg);
  void onRoute(const autoware_planning_msgs::Route::ConstPtr& msg);
  void onTimer(const ros::TimerEvent& event);

  autoware_planning_msgs::Scenario selectScenario();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Timer timer_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  ros::Subscriber sub_lanelet_map_;
  ros::Subscriber sub_route_;

  Input input_lane_following_;
  Input input_parking_;

  Output output_;

  autoware_planning_msgs::Route::ConstPtr route_;
  autoware_planning_msgs::Scenario::_current_scenario_type current_scenario_;
  geometry_msgs::PoseStamped::ConstPtr current_pose_;

  std::shared_ptr<lanelet::LaneletMap> lanelet_map_ptr_;
  std::shared_ptr<lanelet::routing::RoutingGraph> routing_graph_ptr_;
  std::shared_ptr<lanelet::traffic_rules::TrafficRules> traffic_rules_ptr_;
};

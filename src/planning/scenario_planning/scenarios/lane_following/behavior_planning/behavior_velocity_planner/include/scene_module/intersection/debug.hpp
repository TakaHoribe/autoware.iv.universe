#pragma once

#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.hpp>

namespace behavior_planning {

class IntersectionModuleDebugger {
 public:
  IntersectionModuleDebugger();

  void publishLaneletsArea(const std::vector<lanelet::ConstLanelet>& lanelets, const std::string& ns);
  void publishPath(const autoware_planning_msgs::PathWithLaneId& path, const std::string& ns, const double r,
                   const double g, const double b);
  void publishPose(const geometry_msgs::Pose& pose, const std::string& ns, const double r, const double g,
                   const double b, const int mode);
  void publishDebugValues(const std_msgs::Float32MultiArray& msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher debug_viz_pub_;
  ros::Publisher debug_values_pub_;
};

}  // namespace behavior_planning

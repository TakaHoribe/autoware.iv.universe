#pragma once

#include <string>
#include <vector>

#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObject.h>
#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float32MultiArray.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/utilities.h>
#include <lanelet2_routing/RoutingGraph.h>

#include <scene_module/scene_module_interface.h>

class BlindSpotModuleDebugger {
 public:
  BlindSpotModuleDebugger();

  void publishDetectionArea(const std::vector<std::vector<geometry_msgs::Point>>& detection_area, int mode,
                            const std::string& ns);
  void publishPath(const autoware_planning_msgs::PathWithLaneId& path, const std::string& ns, double r, double g,
                   double b);
  void publishPose(const geometry_msgs::Pose& pose, const std::string& ns, double r, double g, double b, int mode);
  void publishDebugValues(const std_msgs::Float32MultiArray& msg);
  void publishGeofence(const geometry_msgs::Pose& pose, int32_t lane_id);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher debug_viz_pub_;
  ros::Publisher debug_values_pub_;
};

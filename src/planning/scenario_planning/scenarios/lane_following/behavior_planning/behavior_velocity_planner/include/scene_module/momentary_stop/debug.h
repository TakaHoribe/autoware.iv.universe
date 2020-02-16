#pragma once

#include <string>

#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

class MomentaryStopDebugMarkersManager {
 public:
  MomentaryStopDebugMarkersManager();
  ~MomentaryStopDebugMarkersManager(){};
  void pushStopPose(const geometry_msgs::Pose& pose);

  void publish();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher debug_viz_pub_;

  std::vector<geometry_msgs::Pose> stop_poses_;
};

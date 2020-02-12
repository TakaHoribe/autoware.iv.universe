#pragma once
#include <autoware_traffic_light_msgs/TrafficLightState.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
// #include <tf2_ros/transform_listener.h>
#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>

namespace behavior_planning {
class TrafficLightDebugMarkersManager {
 public:
  TrafficLightDebugMarkersManager();
  ~TrafficLightDebugMarkersManager(){};
  void pushTrafficLightState(const std::shared_ptr<lanelet::TrafficLight const>& traffic_light,
                             const autoware_traffic_light_msgs::TrafficLightState& state);
  void pushStopPose(const geometry_msgs::Pose& pose);
  void pushJudgePose(const geometry_msgs::Pose& pose);

  void publish();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher debug_viz_pub_;
  // tf2_ros::Buffer tf_buffer_;
  // tf2_ros::TransformListener tf_listener_;

  std::vector<std::tuple<std::shared_ptr<lanelet::TrafficLight const>, autoware_traffic_light_msgs::TrafficLightState>>
      tl_state_;
  std::vector<geometry_msgs::Pose> stop_poses_;
  std::vector<geometry_msgs::Pose> judge_poses_;
};

}  // namespace behavior_planning
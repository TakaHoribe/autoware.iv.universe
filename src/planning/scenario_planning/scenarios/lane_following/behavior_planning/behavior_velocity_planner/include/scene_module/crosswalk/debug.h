#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

#include <ros/ros.h>

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <sensor_msgs/PointCloud2.h>

#include <lanelet2_core/LaneletMap.h>
#include <lanelet2_extension/utility/query.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>

#include <scene_module/scene_module_interface.h>

class CrosswalkDebugMarkersManager {
 public:
  CrosswalkDebugMarkersManager();

  void pushCollisionLine(const std::vector<Eigen::Vector3d>& line);
  void pushCollisionLine(const std::vector<Eigen::Vector2d>& line);
  void pushCollisionPoint(const Eigen::Vector3d& point);
  void pushCollisionPoint(const Eigen::Vector2d& point);
  void pushStopPose(const geometry_msgs::Pose& pose);
  void pushSlowPose(const geometry_msgs::Pose& pose);
  void pushCrosswalkPolygon(const std::vector<Eigen::Vector3d>& polygon);
  void pushCrosswalkPolygon(const std::vector<Eigen::Vector2d>& polygon);
  void pushStopPolygon(const std::vector<Eigen::Vector3d>& polygon);
  void pushStopPolygon(const std::vector<Eigen::Vector2d>& polygon);
  void pushSlowPolygon(const std::vector<Eigen::Vector3d>& polygon);
  void pushSlowPolygon(const std::vector<Eigen::Vector2d>& polygon);

  void publish();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher debug_viz_pub_;
  std::vector<Eigen::Vector3d> collision_points_;
  std::vector<geometry_msgs::Pose> stop_poses_;
  std::vector<geometry_msgs::Pose> slow_poses_;
  std::vector<std::vector<Eigen::Vector3d>> collision_lines_;
  std::vector<std::vector<Eigen::Vector3d>> crosswalk_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> stop_polygons_;
  std::vector<std::vector<Eigen::Vector3d>> slow_polygons_;
};

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

class CrosswalkModule : public SceneModuleInterface {
 public:
  explicit CrosswalkModule(const int64_t module_id, const lanelet::ConstLanelet& crosswalk);

  bool modifyPathVelocity(autoware_planning_msgs::PathWithLaneId* path) override;

 private:
  bool getBackwordPointFromBasePoint(const Eigen::Vector2d& line_point1, const Eigen::Vector2d& line_point2,
                                     const Eigen::Vector2d& base_point, const double backward_length,
                                     Eigen::Vector2d& output_point);

  bool checkSlowArea(
      const autoware_planning_msgs::PathWithLaneId& input,
      const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false>& polygon,
      const autoware_perception_msgs::DynamicObjectArray::ConstPtr& objects_ptr,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& no_ground_pointcloud_ptr,
      autoware_planning_msgs::PathWithLaneId& output);

  bool checkStopArea(
      const autoware_planning_msgs::PathWithLaneId& input,
      const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false>& polygon,
      const autoware_perception_msgs::DynamicObjectArray::ConstPtr& objects_ptr,
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& no_ground_pointcloud_ptr,
      autoware_planning_msgs::PathWithLaneId& output);

  bool insertTargetVelocityPoint(
      const autoware_planning_msgs::PathWithLaneId& input,
      const boost::geometry::model::polygon<boost::geometry::model::d2::point_xy<double>, false>& polygon,
      const double& margin, const double& velocity, autoware_planning_msgs::PathWithLaneId& output);

  enum class State { APPROARCH, INSIDE, GO_OUT };

  lanelet::ConstLanelet crosswalk_;
  State state_;

  // Parameter
  const double stop_margin_ = 1.0;
  const double stop_dynamic_object_prediction_time_margin_ = 3.0;
  const double slow_margin_ = 2.0;
};

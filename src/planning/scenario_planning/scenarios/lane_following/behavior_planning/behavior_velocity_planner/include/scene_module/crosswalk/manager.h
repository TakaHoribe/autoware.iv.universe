#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/lexical_cast.hpp>

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

#include <scene_module/crosswalk/scene.h>
#include <scene_module/scene_module_interface.h>

class CrosswalkModuleManager : public SceneModuleManagerInterface {
 public:
  const char* getModuleName() override { return "crosswalk"; }
  void launchNewModules(const autoware_planning_msgs::PathWithLaneId& path) override;
  void deleteExpiredModules(const autoware_planning_msgs::PathWithLaneId& path) override;
};

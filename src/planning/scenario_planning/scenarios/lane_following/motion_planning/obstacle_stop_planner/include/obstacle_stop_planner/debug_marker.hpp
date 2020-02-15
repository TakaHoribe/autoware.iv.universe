#pragma once
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <memory>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pcl/point_types.h>
#include <string>
#define EIGEN_MPL2_ONLY
#include <Eigen/Core>
#include <Eigen/Geometry>
namespace motion_planning {
class ObstacleStopPlannerDebugNode {
 public:
  ObstacleStopPlannerDebugNode();
  ~ObstacleStopPlannerDebugNode(){};
  void pushPolygon(const std::vector<cv::Point2d>& polygon, const double z);
  void pushPolygon(const std::vector<Eigen::Vector3d>& polygon);
  void pushCollisionPolygon(const std::vector<cv::Point2d>& polygon, const double z);
  void pushCollisionPolygon(const std::vector<Eigen::Vector3d>& polygon);
  void pushStopPose(const geometry_msgs::Pose& stop_pose);
  void pushStopObstaclePoint(const geometry_msgs::Point& stop_obstacle_point);
  void pushStopObstaclePoint(const pcl::PointXYZ& stop_obstacle_point);

  void publish();

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher debug_viz_pub_;

  std::shared_ptr<geometry_msgs::Pose> stop_pose_ptr_;
  std::shared_ptr<geometry_msgs::Point> stop_obstacle_point_ptr_;
  std::vector<std::vector<Eigen::Vector3d>> polygons_;
  std::vector<std::vector<Eigen::Vector3d>> collision_polygons_;
};

}  // namespace motion_planning
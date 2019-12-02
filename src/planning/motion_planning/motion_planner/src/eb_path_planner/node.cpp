
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>



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
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_planner_debug_markes", 1, true);
  path_pub_ = nh_.advertise<autoware_planning_msgs::Path>("planning/motion_planning/avoiding_path", 1, true);
}

EBPathPlannerNode::~EBPathPlannerNode() {}

void EBPathPlannerNode::callback(const autoware_planning_msgs::Path &input_path_msg,
                                 autoware_planning_msgs::Trajectory &output_trajectory_msg)
{
  geometry_msgs::Pose self_pose;
  if (!getSelfPoseInMap(self_pose))
    return;
  
  if(input_path_msg.points.size()==0)
  {
    std::cerr  <<__func__  << "[WARNING] input path size is 0"<< std::endl;
  }

  double min_dist = 999999;
  size_t min_index = 0;
  for (size_t i = 0; i < input_path_msg.points.size(); i++)
  {
    double dx = self_pose.position.x - input_path_msg.points[i].pose.position.x;
    double dy = self_pose.position.y - input_path_msg.points[i].pose.position.y;
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    if (dist < min_dist)
    {
      min_dist = dist;
      min_index = i;
    }
  }
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  std::vector<double> tmp_z;
  std::vector<double> tmp_v;
  for (size_t i = min_index; i < min_index + 50; i++)
  {
    tmp_x.push_back(input_path_msg.points[i].pose.position.x);
    tmp_y.push_back(input_path_msg.points[i].pose.position.y);
    tmp_z.push_back(input_path_msg.points[i].pose.position.z);
    tmp_v.push_back(input_path_msg.points[i].twist.linear.x);
  }
  // ReferencePath reference_path(tmp_x, tmp_y, 0.1);
  // ReferenceTrajectoryPath reference_trajectory_path(tmp_x, tmp_y, tmp_v, 0.1);
  Reference3DTrajectoryPath reference_trajectory_path(tmp_x, tmp_y, tmp_z, tmp_v, 0.1);

  autoware_planning_msgs::Path path_msg;
  path_msg.header.frame_id = "map";
  path_msg.header.stamp = ros::Time::now();
  output_trajectory_msg.header = path_msg.header;
  for (size_t i = 0; i < reference_trajectory_path.x_.size(); i++)
  {
    autoware_planning_msgs::PathPoint path_point_msg;
    path_point_msg.pose.position.x = reference_trajectory_path.x_[i];
    path_point_msg.pose.position.y = reference_trajectory_path.y_[i];
    path_point_msg.pose.position.z = reference_trajectory_path.z_[i];
    float roll = 0;
    float pitch = 0;
    float yaw = reference_trajectory_path.yaw_[i];
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    path_point_msg.pose.orientation = tf2::toMsg(quaternion);
    path_point_msg.twist.linear.x = reference_trajectory_path.v_[i];
    path_msg.points.push_back(path_point_msg);

    autoware_planning_msgs::TrajectoryPoint traj_point_msg;
    traj_point_msg.pose = path_point_msg.pose;
    traj_point_msg.twist = path_point_msg.twist;
    output_trajectory_msg.points.push_back(traj_point_msg);
  }
  path_pub_.publish(path_msg);

  //debug; marker array
  visualization_msgs::MarkerArray marker_array;
  int unique_id = 0;

  // visualize cubic spline point
  visualization_msgs::Marker debug_cubic_spline;
  debug_cubic_spline.lifetime = ros::Duration(1.0);
  debug_cubic_spline.header = path_msg.header;
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
  for (size_t i = 0; i < reference_trajectory_path.x_.size(); i++)
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
} // namespace motion_planner
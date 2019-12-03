
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>



#include <tf2/utils.h>

#include <memory>

#include "eb_path_planner/reference_path.hpp"

#include "eb_path_planner/node.hpp"

namespace motion_planner
{
  
double sign(double a){
  if(a>0) return 1;
  else if(a<0) return -1;
  else return 0;
}

EBPathPlannerNode::EBPathPlannerNode()
    : nh_(),
      private_nh_("~")
{
  markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("path_planner_debug_markes", 1, true);
  path_pub_ = nh_.advertise<autoware_planning_msgs::Path>("planning/motion_planning/avoiding_path", 1, true);
  
  twist_sub_ = private_nh_.subscribe("/current_velocity", 1, 
                           &EBPathPlannerNode::currentVelocityCallback, this);
  private_nh_.param<bool>("enable_velocity_based_cropping", 
                         enable_velocity_based_cropping_,false);
  private_nh_.param<double>("time_for_calculating_velocity_based_distance", 
                    time_for_calculating_velocity_based_distance_, 5);
  private_nh_.param<double>("distance_for_cropping", 
                             distance_for_cropping_, -3);
  
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
    std::cerr  <<"[WARNING] input path size is 0"<< std::endl;
  }

  double distance_threshold = 0;
  if(enable_velocity_based_cropping_)
  {
    if(!in_twist_ptr_)
    {
      std::cerr << "[WARNING] current velocity has not been subscribed"  << std::endl;
      return;
    }
    distance_threshold = in_twist_ptr_->twist.linear.x*
                          time_for_calculating_velocity_based_distance_;
  }
  else
  {
    distance_threshold = distance_for_cropping_;
  }
  
  double yaw = tf2::getYaw(self_pose.orientation);
  double min_dist = 999999;
  size_t min_index = 0;
  for (size_t i = 0; i < input_path_msg.points.size(); i++)
  {
    double dx = input_path_msg.points[i].pose.position.x-self_pose.position.x;
    double dy = input_path_msg.points[i].pose.position.y-self_pose.position.y;
    double dist = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    double ex = self_pose.position.x + std::cos(yaw);
    double ey = self_pose.position.y + std::sin(yaw);
    double inner_product = dx*ex+dy*ey; 
    if(std::abs(distance_threshold)<0.1)
    {
      if (dist < min_dist )
      {
        min_dist = dist;
        min_index = i;
      }
    }
    else
    {
      if (dist < min_dist && 
          dist > std::abs(distance_threshold) &&
         sign(distance_threshold)*inner_product > 0)
      {
        min_dist = dist;
        min_index = i;
      }
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

void EBPathPlannerNode::currentVelocityCallback(const geometry_msgs::TwistStamped& msg)
{
  in_twist_ptr_ = std::make_shared<geometry_msgs::TwistStamped>(msg);
}
} // namespace motion_planner
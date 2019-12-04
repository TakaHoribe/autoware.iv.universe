

#include <tf2/utils.h>

#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>
#include "path2trajectory_converter/node.hpp"

#include "eb_path_planner/reference_path.hpp"

namespace motion_planner
{

Path2Trajectory::Path2Trajectory()
{  
}

Path2Trajectory::~Path2Trajectory() {}

void Path2Trajectory::callback(
  const autoware_planning_msgs::Path &input_path_msg,
        autoware_planning_msgs::Trajectory &output_trajectory_msg)
{
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  std::vector<double> tmp_z;
  std::vector<double> tmp_v;
  for(const auto& path_point: input_path_msg.points)
  {
    tmp_x.push_back(path_point.pose.position.x);
    tmp_y.push_back(path_point.pose.position.y);
    tmp_z.push_back(path_point.pose.position.z);
    tmp_v.push_back(path_point.twist.linear.x);
  }
  if(input_path_msg.points.size()>200)
  {
    ROS_WARN("[Path2Trajectory] Path size <%d> is large. Interpolation takes some time.", 
              input_path_msg.points.size());
  }
  Reference3DTrajectoryPath trajectory(tmp_x, tmp_y, tmp_z, tmp_v, 0.1);
  for(size_t i = 0; i < trajectory.x_.size(); i++)
  {
    autoware_planning_msgs::TrajectoryPoint traj_point;
    traj_point.pose.position.x = trajectory.x_[i]; 
    traj_point.pose.position.y = trajectory.y_[i]; 
    traj_point.pose.position.z = trajectory.z_[i]; 
    float roll = 0;
    float pitch = 0;
    float yaw = trajectory.yaw_[i];
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    traj_point.pose.orientation = tf2::toMsg(quaternion);
    traj_point.twist.linear.x = trajectory.v_[i];
    output_trajectory_msg.points.push_back(traj_point);
  }
  output_trajectory_msg.header = input_path_msg.header;

}
} // namespace motion_planner
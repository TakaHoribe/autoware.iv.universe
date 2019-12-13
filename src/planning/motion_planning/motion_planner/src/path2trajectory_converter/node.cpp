

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
  if(input_path_msg.points.size()>200)
  {
    ROS_WARN("[Path2Trajectory] Path size <%zu> is large. Interpolation could take some time.", 
              input_path_msg.points.size());
  }
  std::vector<double> tmp_x;
  std::vector<double> tmp_y;
  std::vector<double> tmp_z;
  std::vector<double> tmp_v;
  for(size_t i = 0; i <  input_path_msg.points.size(); i++)
  {
    //resolution check
    if(i > 0)
    {
      double dx = input_path_msg.points[i].pose.position.x - 
                  input_path_msg.points[i-1].pose.position.x;
      double dy = input_path_msg.points[i].pose.position.y - 
                  input_path_msg.points[i-1].pose.position.y;
      double dist = std::sqrt(dx*dx + dy*dy); 
      if(dist < 0.1)
      {
        continue;
      }
    }
    //backward check
    if(i > 1)
    {
      double dx1 = input_path_msg.points[i].pose.position.x - 
                  input_path_msg.points[i-1].pose.position.x;
      double dy1 = input_path_msg.points[i].pose.position.y - 
                  input_path_msg.points[i-1].pose.position.y;
      double dx2 = input_path_msg.points[i-1].pose.position.x - 
                  input_path_msg.points[i-2].pose.position.x;
      double dy2 = input_path_msg.points[i-1].pose.position.y - 
                  input_path_msg.points[i-2].pose.position.y;
      double inner_product = dx1*dx2 + dy1*dy2;
      if(inner_product < 0)
      {
        ROS_WARN("Path points might go backward");
      }
    }
    tmp_x.push_back(input_path_msg.points[i].pose.position.x);
    tmp_y.push_back(input_path_msg.points[i].pose.position.y);
    tmp_z.push_back(input_path_msg.points[i].pose.position.z);
    tmp_v.push_back(input_path_msg.points[i].twist.linear.x);
  }
  
  Spline3D spline3d(tmp_x, tmp_y, tmp_z);
  for(double s = 0.1; s < spline3d.s.back(); s+=0.1)
  {
    autoware_planning_msgs::TrajectoryPoint traj_point;
    std::array<double, 3> point = spline3d.calc_point(s);
    traj_point.pose.position.x = point[0]; 
    traj_point.pose.position.y = point[1]; 
    traj_point.pose.position.z = point[2]; 
    float roll = 0;
    float pitch = 0;
    float yaw = spline3d.calc_yaw(s);
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    traj_point.pose.orientation = tf2::toMsg(quaternion);
    auto it = std::lower_bound(spline3d.s.begin(), 
                               spline3d.s.end(),
                               s) - 1;
    size_t ind = std::distance(spline3d.s.begin(), it);
    traj_point.twist.linear.x = tmp_v[ind];
    output_trajectory_msg.points.push_back(traj_point); 
  }
  output_trajectory_msg.header = input_path_msg.header;

}
} // namespace motion_planner
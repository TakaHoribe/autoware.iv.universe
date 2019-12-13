

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
        ROS_INFO("Path points might go backward");
      }
    }
    
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
        if(input_path_msg.points[i].twist.linear.x < 
           input_path_msg.points[i-1].twist.linear.x)
        {
          //replace point with lower velocity
          tmp_x.pop_back();
          tmp_y.pop_back();
          tmp_z.pop_back();
          tmp_v.pop_back();
          tmp_x.push_back(input_path_msg.points[i].pose.position.x);
          tmp_y.push_back(input_path_msg.points[i].pose.position.y);
          tmp_z.push_back(input_path_msg.points[i].pose.position.z);
          tmp_v.push_back(input_path_msg.points[i].twist.linear.x);
          continue;
        }
        else
        {
          continue;
        }
      }
    }
    tmp_x.push_back(input_path_msg.points[i].pose.position.x);
    tmp_y.push_back(input_path_msg.points[i].pose.position.y);
    tmp_z.push_back(input_path_msg.points[i].pose.position.z);
    tmp_v.push_back(input_path_msg.points[i].twist.linear.x);
  }
  
  Spline2D spline2d(tmp_x, tmp_y);
  for(double s = 0.1; s < spline2d.s.back(); s+=0.1)
  {
    autoware_planning_msgs::TrajectoryPoint traj_point;
    std::array<double, 2> point = spline2d.calc_position(s);
    traj_point.pose.position.x = point[0]; 
    traj_point.pose.position.y = point[1]; 
    if(isnan(point[0]) || isnan(point[1]))
    {
      ROS_ERROR("[path2tracjectory]: Interpolation gets nan value. Relay path to trajectory, but point interval is more than 0.1m");
      msgConversionFromPath2Trajectory(input_path_msg, output_trajectory_msg); 
      return;
    }
    float roll = 0;
    float pitch = 0;
    float yaw = spline2d.calc_yaw(s);
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    traj_point.pose.orientation = tf2::toMsg(quaternion);
    auto it = std::lower_bound(spline2d.s.begin(), 
                               spline2d.s.end(),
                               s);
    size_t ind = std::distance(spline2d.s.begin(), it);
    traj_point.twist.linear.x = tmp_v[ind];
    traj_point.pose.position.z = tmp_z[ind];
    output_trajectory_msg.points.push_back(traj_point); 
  }
  
  //sanity check for last point
  if(tmp_v.back()<1e-8 &&
     output_trajectory_msg.points.back().twist.linear.x > 1e-8)
  {
    output_trajectory_msg.points.back().twist.linear.x = 0;
  }
  
  output_trajectory_msg.header = input_path_msg.header;
}

void Path2Trajectory::msgConversionFromPath2Trajectory(
    const autoware_planning_msgs::Path& path,
    autoware_planning_msgs::Trajectory& traj)
{
  traj.header = path.header;
  for(const auto& point: path.points)
  {
    autoware_planning_msgs::TrajectoryPoint traj_point;
    traj_point.pose = point.pose;
    traj_point.twist = point.twist;
    traj.points.push_back(traj_point);
  }
}
} // namespace motion_planner
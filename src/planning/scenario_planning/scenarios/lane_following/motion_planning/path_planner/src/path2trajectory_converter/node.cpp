

#include <tf2/utils.h>

#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/Trajectory.h>
#include "path2trajectory_converter/node.hpp"

#include "path2trajectory_converter/horibe_interpolate.h"
// #include "eb_path_planner/reference_path.hpp"

namespace path_planner
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
  
  std::vector<double> base_s = horibe_spline::calcEuclidDist(tmp_x, tmp_y);
  std::vector<double> new_s;
  for(double i = 0.1; 
      i <= base_s.back();
      i += 0.1)
  {
    new_s.push_back(i);
  }
  horibe_spline::SplineInterpolate spline;
  std::vector<double> new_x;
  std::vector<double> new_y;
  spline.interpolate(base_s, tmp_x, new_s, new_x);
  spline.interpolate(base_s, tmp_y, new_s, new_y);
  for(size_t i = 0; i < new_s.size(); i++)
  {
    autoware_planning_msgs::TrajectoryPoint traj_point;
    traj_point.pose.position.x = new_x[i]; 
    traj_point.pose.position.y = new_y[i]; 
    if(std::isnan(new_x[i]) || std::isnan(new_y[i]))
    {
      ROS_WARN("[path2tracjectory]: Interpolation gets nan value. Relay path to trajectory, but point interval is more than 0.1m");
      msgConversionFromPath2Trajectory(input_path_msg, output_trajectory_msg); 
      return;
    }
    double roll = 0;
    double pitch = 0;
    double yaw = 0;
    if(i==new_s.size()-1)
    {
      double dx = new_x[i] - new_x[i-1]; 
      double dy = new_y[i] - new_y[i-1]; 
      yaw = std::atan2(dy, dx);
    }
    else
    {
      double dx = new_x[i+1] - new_x[i]; 
      double dy = new_y[i+1] - new_y[i]; 
      yaw = std::atan2(dy, dx);
    }
    tf2::Quaternion quaternion;
    quaternion.setRPY( roll, pitch, yaw );
    traj_point.pose.orientation = tf2::toMsg(quaternion);
    auto it = std::lower_bound(base_s.begin(), 
                               base_s.end(),
                               new_s[i]);
    size_t ind = std::distance(base_s.begin(), it);
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
} // namespace path_planner

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
  output_trajectory_msg.header = input_path_msg.header;
  for(const auto& path: input_path_msg.points)
  {
    autoware_planning_msgs::TrajectoryPoint traj_point;
    traj_point.pose = path.pose;
    traj_point.twist = path.twist;
    output_trajectory_msg.points.push_back(traj_point);
  }
}
} // namespace motion_planner
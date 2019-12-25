#pragma once
#include <utilization/interpolation/cubic_spline.hpp>
#include <autoware_planning_msgs/Path.h>
namespace behavior_planning
{
void interpolatePath(const autoware_planning_msgs::Path &path, const double length, autoware_planning_msgs::Path &interpolated_path);
void filterLitterPathPoint(const autoware_planning_msgs::Path &path, autoware_planning_msgs::Path &filtered_path);
void filterStopPathPoint(const autoware_planning_msgs::Path &path, autoware_planning_msgs::Path &filtered_path);
bool calcClosestIdx(const autoware_planning_msgs::Path &path, const geometry_msgs::Pose &curr_pose, int &closest, double dist_thr = 3.0, double angle_thr = M_PI_2);
} // namespace behavior_planning

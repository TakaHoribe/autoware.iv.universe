#pragma once
#include <utilization/interporation/cubic_spline.hpp>
#include <autoware_planning_msgs/Path.h>
namespace behavior_planning
{
void interporatePath(const autoware_planning_msgs::Path &path, const double length, autoware_planning_msgs::Path &interporated_path);
void filterLitterPathPoint(const autoware_planning_msgs::Path &path, autoware_planning_msgs::Path &filtered_path);
void filterStopPathPoint(const autoware_planning_msgs::Path &path, autoware_planning_msgs::Path &filtered_path);
} // namespace behavior_planning

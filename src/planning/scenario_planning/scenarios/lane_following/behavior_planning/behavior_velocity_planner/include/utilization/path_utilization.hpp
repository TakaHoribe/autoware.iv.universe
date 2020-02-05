#pragma once
#include <autoware_planning_msgs/Path.h>
#include <utilization/interpolation/cubic_spline.hpp>
namespace behavior_planning {
void interpolatePath(const autoware_planning_msgs::Path& path, const double length,
                     autoware_planning_msgs::Path& interpolated_path);
void filterLitterPathPoint(const autoware_planning_msgs::Path& path, autoware_planning_msgs::Path& filtered_path);
void filterStopPathPoint(const autoware_planning_msgs::Path& path, autoware_planning_msgs::Path& filtered_path);
}  // namespace behavior_planning

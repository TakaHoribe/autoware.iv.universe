#pragma once

#include <autoware_planning_msgs/Path.h>

autoware_planning_msgs::Path interpolatePath(const autoware_planning_msgs::Path& path, const double length);
autoware_planning_msgs::Path filterLitterPathPoint(const autoware_planning_msgs::Path& path);
autoware_planning_msgs::Path filterStopPathPoint(const autoware_planning_msgs::Path& path);

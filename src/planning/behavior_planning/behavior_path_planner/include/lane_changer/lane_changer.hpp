#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Path.h>
#include <memory>

namespace behavior_planning
{
class LaneChanger
{
public:
    LaneChanger(const double search_radius_range,
                    const double search_rad_range, 
                    const autoware_planning_msgs::Path &path,
                    const geometry_msgs::Pose &goal);
    LaneChanger();
};
} // namespace behavior_planning
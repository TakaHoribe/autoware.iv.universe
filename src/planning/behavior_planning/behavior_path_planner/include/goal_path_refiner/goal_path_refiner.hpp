#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Path.h>
#include <memory>

namespace behavior_planning
{
class GoalPathRefiner
{
public:
    GoalPathRefiner(const double search_radius_range,
                    const double search_rad_range, 
                    const autoware_planning_msgs::Path &path,
                    const geometry_msgs::Pose &goal);
    GoalPathRefiner();
    bool getRefinedPath(autoware_planning_msgs::Path &output);
    static bool getRefinedPath(const double search_radius_range,
                               const double search_rad_range,
                               const autoware_planning_msgs::Path &input,
                               const geometry_msgs::Pose &goal,
                               autoware_planning_msgs::Path &output);

private:
    double radius_threshold_;
    double rad_threshold_;
    std::shared_ptr<autoware_planning_msgs::Path> path_ptr_;
    std::shared_ptr<geometry_msgs::Pose> goal_ptr_;

};
} // namespace behavior_planning
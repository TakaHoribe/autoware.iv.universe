#include <goal_path_refiner/goal_path_refiner.hpp>

#include <autoware_planning_msgs/Route.h>
#include <autoware_planning_msgs/Path.h>

namespace behavior_planning
{
GoalPathRefiner::GoalPathRefiner(const double search_radius_range,
                                 const double search_rad_range,
                                 const autoware_planning_msgs::Path &path,
                                 const geometry_msgs::Pose &goal) : radius_threshold_(search_radius_range),
                                                                    rad_threshold_(search_rad_range)
{
    path_ptr_ = std::make_shared<autoware_planning_msgs::Path>(path);
    goal_ptr_ = std::make_shared<geometry_msgs::Pose>(goal);
}

GoalPathRefiner::GoalPathRefiner(){};

bool GoalPathRefiner::getRefinedPath(autoware_planning_msgs::Path &output)
{
    if (path_ptr_ == nullptr || goal_ptr_ == nullptr)
        return false;
    return getRefinedPath(radius_threshold_, rad_threshold_, *path_ptr_, *goal_ptr_, output);
}

bool GoalPathRefiner::getRefinedPath(const double search_radius_range, const double search_rad_range, const autoware_planning_msgs::Path &input,
                                     const geometry_msgs::Pose &goal,
                                     autoware_planning_msgs::Path &output)
{
    bool found = false;
    size_t min_dist_index;
    double min_dist;
    for (size_t i = 0; i < input.points.size(); ++i)
    {
        const double x = input.points.at(i).pose.position.x - goal.position.x;
        const double y = input.points.at(i).pose.position.y - goal.position.y;
        const double z = input.points.at(i).pose.position.z - goal.position.z;
        const double dist = sqrt(x * x + y * y + z * z);
        if (dist < min_dist || i == 0 /*init*/)
        {
            min_dist_index = i;
            min_dist = dist;
            found = true;
        }
    }
    autoware_planning_msgs::PathPoint refined_goal;
    refined_goal.pose = goal;
    refined_goal.twist.linear.x = 0.0;
    if (!found)
        return false;
    output.points.erase(output.points.begin() + min_dist_index, output.points.end());
    output.points.push_back(refined_goal);
    return true;
}

} // namespace behavior_planning
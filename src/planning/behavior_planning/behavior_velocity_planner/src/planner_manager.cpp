
#include <behavior_velocity_planner/planner_manager.hpp>

namespace behavior_planning
{

BehaviorVelocityPlannerManager::BehaviorVelocityPlannerManager() {}

bool BehaviorVelocityPlannerManager::callback(const autoware_planning_msgs::PathWithLaneId &input_path_msg,
                                              autoware_planning_msgs::PathWithLaneId &output_path_msg)
{
    for (const auto &path_point : input_path_msg.points)
    {
        output_path_msg.points.push_back(path_point);
    }
    return true;
}
} // namespace behavior_planning
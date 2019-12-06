#include <behavior_path_planner/cache_path_manager.hpp>

namespace behavior_planning
{
CachePathManager::CachePathManager(){};
void CachePathManager::reset()
{
    cached_path_ptr_ = nullptr;
}
bool CachePathManager::empty()
{
    if (cached_path_ptr_ == nullptr)
        return true;
    else
        return false;
}

// std::shared_ptr<autoware_planning_msgs::PathWithLaneId> cached_path_ptr_;
// autoware_planning_msgs::PathPointWithLaneId last_path_point_;
} // namespace behavior_planning
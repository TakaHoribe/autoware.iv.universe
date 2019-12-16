#pragma once
#include <autoware_planning_msgs/PathWithLaneId.h>
#include <memory>
namespace behavior_planning
{
class CachePathManager
{
public:
    CachePathManager();
    void reset();
    bool empty();

private:
    std::shared_ptr<autoware_planning_msgs::PathWithLaneId> cached_path_ptr_;
    autoware_planning_msgs::PathPointWithLaneId last_path_point_;
};
} // namespace behavior_planning
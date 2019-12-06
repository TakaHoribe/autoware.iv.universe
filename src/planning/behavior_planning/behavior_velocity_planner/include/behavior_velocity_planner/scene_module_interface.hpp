#pragma once
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>

namespace behavior_planning
{

class SceneModuleInterface
{
public:
    SceneModuleInterface(){};
    virtual bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output) = 0;
    virtual bool endOfLife(const autoware_planning_msgs::PathWithLaneId &input) = 0;
    virtual ~SceneModuleInterface(){};
};

class SceneConditionInterface
{
public:
    SceneConditionInterface(){};
    virtual bool startCondition(const autoware_planning_msgs::PathWithLaneId &input, std::shared_ptr<SceneModuleInterface> &module_ptr) = 0;
    virtual ~SceneConditionInterface(){};
};

} // namespace behavior_planning
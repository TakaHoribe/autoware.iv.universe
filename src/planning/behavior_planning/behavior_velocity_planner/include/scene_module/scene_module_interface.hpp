#pragma once
#include <autoware_planning_msgs/Path.h>
#include <autoware_planning_msgs/PathWithLaneId.h>

namespace behavior_planning
{

class SceneModuleInterface
{
public:
    virtual bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output) = 0;
    virtual bool endOfLife(const autoware_planning_msgs::PathWithLaneId &input) = 0;

public:
    SceneModuleInterface(){};
    virtual ~SceneModuleInterface(){};
};

class SceneModuleManagerInterface
{
public:
    virtual bool startCondition(const autoware_planning_msgs::PathWithLaneId &input, std::vector<std::shared_ptr<SceneModuleInterface>> &v_module_ptr) = 0;

public:
    SceneModuleManagerInterface(){};
    virtual bool updateSceneModuleInstances(const autoware_planning_msgs::PathWithLaneId &input)
    {
        // create instance
        std::vector<std::shared_ptr<SceneModuleInterface>> scene_modules_ptr;
        startCondition(input, scene_modules_ptr);
        for (const auto &scene_module_ptr : scene_modules_ptr)
        {
            if (scene_module_ptr != nullptr)
                scene_modules_ptr_.push_back(scene_module_ptr);
        }
        // delete instance
        for (int i = 0; i < (int)scene_modules_ptr_.size(); ++i)
        {
            if (scene_modules_ptr_.at(i)->endOfLife(input))
            {
                scene_modules_ptr_.erase(scene_modules_ptr_.begin() + i);
                i += -1;
            }
        }
        return true;
    };
    virtual bool run(const autoware_planning_msgs::PathWithLaneId &input, autoware_planning_msgs::PathWithLaneId &output){
        autoware_planning_msgs::PathWithLaneId input_path = input;
        for (size_t i = 0; i < scene_modules_ptr_.size(); ++i)
        {
            autoware_planning_msgs::PathWithLaneId output_path;
            if (scene_modules_ptr_.at(i)->run(input_path, output_path))
                input_path = output_path;
        }
        output = input_path;
        return true;
    }
    virtual ~SceneModuleManagerInterface(){};

protected:
    std::vector<std::shared_ptr<SceneModuleInterface>> scene_modules_ptr_;
};

} // namespace behavior_planning
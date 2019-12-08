#include <behavior_velocity_planner/planner_manager.hpp>

// Scene include
#include <scene_module/momentary_stop/momentary_stop.hpp>

namespace behavior_planning
{

BehaviorVelocityPlannerManager::BehaviorVelocityPlannerManager()
{
    v_scene_condition_.push_back(std::make_shared<MomentaryStopCondition>());
}

bool BehaviorVelocityPlannerManager::callback(const autoware_planning_msgs::PathWithLaneId &input_path_msg,
                                              autoware_planning_msgs::PathWithLaneId &output_path_msg)
{
    for (size_t i = 0; i < v_scene_condition_.size(); ++i)
    {
        std::vector<std::shared_ptr<SceneModuleInterface>> v_scene_module_ptr;
        if (v_scene_condition_.at(i)->startCondition(input_path_msg, v_scene_module_ptr))
        {
            for (const auto &scene_module_ptr : v_scene_module_ptr)
            {
                if (scene_module_ptr != nullptr)
                    v_scene_module_.push_back(scene_module_ptr);
            }
        }
    }

    for (int i = 0; i < (int)v_scene_module_.size(); ++i)
    {
        if (v_scene_module_.at(i)->endOfLife(input_path_msg))
        {
            v_scene_module_.erase(v_scene_module_.begin() + i);
            i += -1;
        }
    }

    autoware_planning_msgs::PathWithLaneId input_path = input_path_msg;
    for (size_t i = 0; i < v_scene_module_.size(); ++i)
    {
        autoware_planning_msgs::PathWithLaneId output_path;
        if(v_scene_module_.at(i)->run(input_path, output_path))
            input_path = output_path;
    }

    output_path_msg = input_path;

    return true;
}
} // namespace behavior_planning
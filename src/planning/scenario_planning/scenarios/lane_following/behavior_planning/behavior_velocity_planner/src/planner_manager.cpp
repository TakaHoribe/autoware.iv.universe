#include <behavior_velocity_planner/planner_manager.hpp>

// Scene include
#include <scene_module/momentary_stop/momentary_stop.hpp>
#include <scene_module/crosswalk/crosswalk.hpp>
#include <scene_module/traffic_light/traffic_light.hpp>
#include <scene_module/intersection/intersection.hpp>

namespace behavior_planning
{

BehaviorVelocityPlannerManager::BehaviorVelocityPlannerManager()
{
    scene_managers_ptr_.push_back(std::make_shared<MomentaryStopModuleManager>());
    scene_managers_ptr_.push_back(std::make_shared<CrosswalkModuleManager>());
    scene_managers_ptr_.push_back(std::make_shared<TrafficLightModuleManager>());
    scene_managers_ptr_.push_back(std::make_shared<IntersectionModuleManager>());
}

bool BehaviorVelocityPlannerManager::callback(const autoware_planning_msgs::PathWithLaneId &input_path_msg,
                                              autoware_planning_msgs::PathWithLaneId &output_path_msg)
{
    for (size_t i = 0; i < scene_managers_ptr_.size(); ++i)
    {
        if (!scene_managers_ptr_.at(i)->updateSceneModuleInstances(input_path_msg))
        {
            return false;
        }
    }

    autoware_planning_msgs::PathWithLaneId input_path = input_path_msg;
    for (size_t i = 0; i < scene_managers_ptr_.size(); ++i)
    {
        autoware_planning_msgs::PathWithLaneId output_path;
        if (scene_managers_ptr_.at(i)->run(input_path, output_path))
            input_path = output_path;
    }

    output_path_msg = input_path;

    return true;
}
} // namespace behavior_planning
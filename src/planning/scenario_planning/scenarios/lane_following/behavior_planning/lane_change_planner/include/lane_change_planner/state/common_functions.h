#pragma once

#include <autoware_perception_msgs/DynamicObjectArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <lane_change_planner/state/state_base_class.h>
#include <lanelet2_core/primitives/Primitive.h>
#include <memory>

namespace lane_change_planner
{
namespace state_machine
{
namespace common_functions
{
bool isLaneChangePathSafe(const autoware_planning_msgs::PathWithLaneId& path,
                          const lanelet::ConstLanelets& current_lanes, const lanelet::ConstLanelets& target_lanes,
                          const std::shared_ptr<autoware_perception_msgs::DynamicObjectArray const>& dynamic_objects,
                          const geometry_msgs::Pose& current_pose, const geometry_msgs::Twist& current_twist,
                          const LaneChangerParameters& ros_parameters, const bool use_buffer = true);
}
}  // namespace state_machine
}  // namespace lane_change_planner
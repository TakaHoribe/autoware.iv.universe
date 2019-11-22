#include "behavior_planning_scheduler/node.hpp"
#include <fstream>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace behavior_planner
{
BehaviorPlanningSchedulerNode::BehaviorPlanningSchedulerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
  path_sub_ = nh_.subscribe("input/path", 1, &BehaviorPlanningSchedulerNode::pathCallback, this);
  path_pub_ = nh_.advertise<autoware_planning_msgs::Path>("output/path", 1);
}

void BehaviorPlanningSchedulerNode::pathCallback(const autoware_planning_msgs::Path &input_path_msg)
{
  if (path_pub_.getNumSubscribers() < 1)
    return;
  autoware_planning_msgs::Path output_path_msg;
  output_path_msg.header = input_path_msg.header;
  path_pub_.publish(output_path_msg);
}
}
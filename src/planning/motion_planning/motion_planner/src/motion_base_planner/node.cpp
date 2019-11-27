#include "motion_base_planner/node.hpp"

namespace motion_planner
{
BasePlannerNode::BasePlannerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
  path_sub_ = pnh_.subscribe("input/path", 1, &BasePlannerNode::pathCallback, this);
  path_pub_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/path", 1);
}

void BasePlannerNode::pathCallback(const autoware_planning_msgs::Path &input_path_msg)
{
  if (path_pub_.getNumSubscribers() < 1)
    return;
  if (input_path_msg.points.empty())
    ROS_WARN("path is empty");
  autoware_planning_msgs::Path output_path_msg;
  output_path_msg.header = input_path_msg.header;

  callback(input_path_msg, output_path_msg);
  path_pub_.publish(output_path_msg);
}

bool BasePlannerNode::getSelfPose(geometry_msgs::TransformStamped &self_pose, const std_msgs::Header &header)
{
  try
  {
    self_pose = tf_buffer_.lookupTransform(header.frame_id, "base_link",
                                           header.stamp, ros::Duration(0.1));
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    return false;
  }
}

bool BasePlannerNode::getCurrentSelfPose(geometry_msgs::TransformStamped &self_pose)
{
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time::now();
  return getSelfPose(self_pose, header);
}

} // namespace motion_planner
#include "path_base_planner/node.hpp"

namespace path_planner
{
BasePlannerNode::BasePlannerNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_)
{
  timer_ = nh_.createTimer(ros::Duration(0.1), &BasePlannerNode::timerCallback, this);
  path_sub_ = pnh_.subscribe("input/path", 1, &BasePlannerNode::pathCallback, this);
  trajectory_pub_ = pnh_.advertise<autoware_planning_msgs::Trajectory>("output/path", 1);
}

void BasePlannerNode::timerCallback(const ros::TimerEvent &e){
  if (trajectory_pub_.getNumSubscribers() < 1)
  {
    ROS_WARN_THROTTLE(10.0,"There is no subscribers for Motion Path Planner; Skip callback");
    return;
  }
  if(path_ptr_ == nullptr)
  {
    ROS_INFO("Waiting for /planning/behavior_planning/path");
    return;
  }
  autoware_planning_msgs::Trajectory output_trajectory_msg;
  output_trajectory_msg.header = path_ptr_->header;

  callback(*path_ptr_, output_trajectory_msg);
  trajectory_pub_.publish(output_trajectory_msg);
}


void BasePlannerNode::pathCallback(const autoware_planning_msgs::Path &input_path_msg)
{
  if (input_path_msg.points.empty())
    ROS_WARN("path is empty");
  path_ptr_ = std::make_shared<autoware_planning_msgs::Path>(input_path_msg);
}

bool BasePlannerNode::getSelfPose(geometry_msgs::Pose &self_pose, const std_msgs::Header &header)
{
  try
  {
    geometry_msgs::TransformStamped transform;
    transform = tf_buffer_.lookupTransform(header.frame_id, "base_link",
                                           header.stamp, ros::Duration(0.1));
    self_pose.position.x = transform.transform.translation.x;
    self_pose.position.y = transform.transform.translation.y;
    self_pose.position.z = transform.transform.translation.z;
    self_pose.orientation.x = transform.transform.rotation.x;
    self_pose.orientation.y = transform.transform.rotation.y;
    self_pose.orientation.z = transform.transform.rotation.z;
    self_pose.orientation.w = transform.transform.rotation.w;
    return true;
  }
  catch (tf2::TransformException &ex)
  {
    return false;
  }
}

bool BasePlannerNode::getSelfPoseInMap(geometry_msgs::Pose &self_pose)
{
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time();
  return getSelfPose(self_pose, header);
}

} // namespace path_planner
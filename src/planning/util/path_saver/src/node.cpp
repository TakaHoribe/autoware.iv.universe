#include "path_saver/node.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

PathSaverNode::PathSaverNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
  twist_sub_ = nh_.subscribe("twist", 1, &PathSaverNode::twistCallback, this);
  timer_ = nh_.createTimer(ros::Duration(0.01), &PathSaverNode::timerCallback, this);

  nh_.param("dist_threshold", dist_threshold_, 1.0);
  nh_.param("time_threshold", time_threshold_, 0.1);

  std::string file_name;
  pnh_.getParam("path_file_name", file_name);
  ofs_.open(file_name, std::ios::trunc);
  ofs_ << "x, y, z, roll[rad], pitch[rad], yaw[rad], linear_velocity[m/s], angular_velocity[rad/s]" << std::endl;
}

void PathSaverNode::twistCallback(const geometry_msgs::TwistStamped::ConstPtr& input_twist_msg) {
  twist_ = input_twist_msg;
}

void PathSaverNode::timerCallback(const ros::TimerEvent&) {
  std_msgs::Header header;
  header.frame_id = "map";
  header.stamp = ros::Time::now();
  geometry_msgs::TransformStamped self_pose;
  try {
    self_pose = tf_buffer_.lookupTransform(header.frame_id, "base_link", header.stamp, ros::Duration(0.5));
  } catch (tf2::TransformException& ex) {
    return;
  }

  if (last_saved_time_ == nullptr) {
    last_saved_time_ = std::make_shared<ros::Time>();

  } else {
    const double x = last_saved_pose_.position.x - self_pose.transform.translation.x;
    const double y = last_saved_pose_.position.y - self_pose.transform.translation.y;
    const double z = last_saved_pose_.position.z - self_pose.transform.translation.z;
    const double dist = std::sqrt(x * x + y * y + z * z);
    const double velocity = last_saved_twist_.linear.x;
    const double dist_from_velocity = velocity * time_threshold_;
    if (dist < dist_threshold_ && dist_from_velocity < dist_threshold_) return;
  }

  // save
  double roll, pitch, yaw;
  {
    tf2::Quaternion tf2_quaternion;
    tf2::fromMsg(self_pose.transform.rotation, tf2_quaternion);
    tf2::Matrix3x3 tf2_matrix(tf2_quaternion);
    tf2_matrix.getRPY(roll, pitch, yaw);
  }
  ofs_ << self_pose.transform.translation.x << ", " << self_pose.transform.translation.y << ", "
       << self_pose.transform.translation.z << ", " << roll << ", " << pitch << ", " << yaw << ", "
       << twist_->twist.linear.x << ", " << twist_->twist.angular.z << std::endl;
  // last saved data
  *last_saved_time_ = header.stamp;
  last_saved_pose_.position.x = self_pose.transform.translation.x;
  last_saved_pose_.position.y = self_pose.transform.translation.y;
  last_saved_pose_.position.z = self_pose.transform.translation.z;
  last_saved_pose_.orientation.x = self_pose.transform.rotation.x;
  last_saved_pose_.orientation.y = self_pose.transform.rotation.y;
  last_saved_pose_.orientation.z = self_pose.transform.rotation.z;
  last_saved_pose_.orientation.w = self_pose.transform.rotation.w;
  last_saved_twist_ = twist_->twist;
}

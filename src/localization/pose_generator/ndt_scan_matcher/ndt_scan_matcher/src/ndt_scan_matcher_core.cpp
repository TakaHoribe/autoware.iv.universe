/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "ndt_scan_matcher/ndt_scan_matcher_core.h"

#include <cmath>
#include <iomanip>
#include <algorithm>
#include <thread>

#include <tf/tf.h>
#include <tf2_eigen/tf2_eigen.h>

#include <pcl_conversions/pcl_conversions.h>
#include <eigen_conversions/eigen_msg.h>

#include "ndt_scan_matcher/util_func.h"

NDTScanMatcher::NDTScanMatcher(ros::NodeHandle nh, ros::NodeHandle private_nh)
  : nh_(nh)
  , private_nh_(private_nh)
  , tf2_listener_(tf2_buffer_)
  , base_frame_("base_link")
  , map_frame_("map")
{
  ROS_INFO("use NDT SLAM PCL GENERIC version");
  ndt_ptr_.reset(new pcl::NormalDistributionsTransformModified<PointSource, PointTarget>);

  int points_queue_size = 0;
  private_nh_.getParam("input_sensor_points_queue_size", points_queue_size);
  points_queue_size = std::max(points_queue_size, 0);
  ROS_INFO("points_queue_size: %d", points_queue_size);

  private_nh_.getParam("base_frame", base_frame_);
  ROS_INFO("base_frame_id: %s", base_frame_.c_str());

  double trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  double step_size = ndt_ptr_->getStepSize();
  double resolution = ndt_ptr_->getResolution();
  int max_iterations = ndt_ptr_->getMaximumIterations();
  private_nh_.getParam("trans_epsilon", trans_epsilon);
  private_nh_.getParam("step_size", step_size);
  private_nh_.getParam("resolution", resolution);
  private_nh_.getParam("max_iterations", max_iterations);
  ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setResolution(resolution);
  ndt_ptr_->setMaximumIterations(max_iterations);
  ROS_INFO("trans_epsilon: %lf, step_size: %lf, resolution: %lf, max_iterations: %d",
            trans_epsilon, step_size, resolution, max_iterations);

  initial_pose_sub_ = nh_.subscribe("ekf_pose_with_covariance", 100, &NDTScanMatcher::callbackInitialPose, this);
  map_points_sub_ = nh_.subscribe("pointcloud_map", 1, &NDTScanMatcher::callbackMapPoints, this);
  sensor_points_sub_ = nh_.subscribe("points_raw", 1, &NDTScanMatcher::callbackSensorPoints, this);

  sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);
  ndt_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);
  ndt_pose_with_covariance_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("ndt_pose_with_covariance", 10);
  initial_pose_with_covariance_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("initial_pose_with_covariance", 10);
  exe_time_pub_ = nh.advertise<std_msgs::Float32>("exe_time_ms", 10);
  transform_probability_pub_ = nh.advertise<std_msgs::Float32>("transform_probability", 10);
  iteration_num_pub_ = nh.advertise<std_msgs::Float32>("iteration_num", 10);
  ndt_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("/ndt_marker", 10);

  service_ = nh.advertiseService("ndt_align_srv", &NDTScanMatcher::serviceNDTAlign, this);
  // setup dynamic reconfigure server
  // f_ = boost::bind(&NDTScanMatcher::configCallback, this, _1, _2);
  // server_.setCallback(f_);
}

NDTScanMatcher::~NDTScanMatcher()
{
}

// void NDTScanMatcher::configCallback(const ndt_slam::NDTScanMatcherConfig &config, uint32_t level)
// {
// }

bool NDTScanMatcher::serviceNDTAlign(ndt_scan_matcher::NDTAlign::Request &req, ndt_scan_matcher::NDTAlign::Response &res)
{
  std::cout << req.initial_pose_with_cov.pose.pose.position.x << std::endl;
  res.result_pose_with_cov = req.initial_pose_with_cov;

  //const auto pcdmsg = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("pointcloud_map", ros::Duration(3.0));

  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  if (ndt_ptr_->getInputTarget() == nullptr || ndt_ptr_->getInputSource() == nullptr) {
    ROS_WARN("F***********************************K");
    return false;
  }

  Eigen::Affine3d initial_pose_affine;
  Eigen::fromMsg(req.initial_pose_with_cov.pose.pose, initial_pose_affine);
  const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

  pcl::PointCloud<PointSource>::Ptr output_cloud(new pcl::PointCloud<PointSource>);
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);


  const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg =  tf2::toMsg(result_pose_affine);

  res.converged = true;
  res.result_pose_with_cov.header = res.result_pose_with_cov.header;
  res.result_pose_with_cov.pose.pose = result_pose_msg;
  return true;
}

void NDTScanMatcher::callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &initial_pose_msg_ptr)
{
  initial_pose_msg_ptr_array_.push_back(initial_pose_msg_ptr);

  // if rosbag restart, clear buffer
}


void NDTScanMatcher::callbackSensorPoints(const sensor_msgs::PointCloud2::ConstPtr &sensor_points_sensorTF_msg_ptr)
{

  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(ndt_map_mtx_);

  if (ndt_ptr_->getInputTarget() == nullptr) {
    ROS_WARN_STREAM_THROTTLE(1, "No MAP! F***********************************K");
    return;
  }
  
  const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;
  current_scan_time_ = sensor_ros_time;

  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_sensorTF_ptr(new pcl::PointCloud<PointSource>);
  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);

  // get TF base to sensor
  geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
  getTransform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);
  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

  boost::shared_ptr<pcl::PointCloud<PointSource>> sensor_points_baselinkTF_ptr(new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(*sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);

  ndt_ptr_->setInputSource(sensor_points_baselinkTF_ptr);

  // check
  if (initial_pose_msg_ptr_array_.empty()) {
    ROS_WARN_STREAM_THROTTLE(1, "No Pose! F***********************************K");
    return;
  }

  // searchNNPose
  geometry_msgs::PoseStamped initial_pose_old_msg;
  geometry_msgs::PoseStamped initial_pose_new_msg;
  while (!initial_pose_msg_ptr_array_.empty())
  {
    geometry_msgs::PoseWithCovarianceStamped tmp_pose_cov_msg;
    tmp_pose_cov_msg = *(initial_pose_msg_ptr_array_.front());
    initial_pose_new_msg.header = tmp_pose_cov_msg.header;
    initial_pose_new_msg.pose = tmp_pose_cov_msg.pose.pose;
    initial_pose_msg_ptr_array_.pop_front();
    if (initial_pose_new_msg.header.stamp > sensor_ros_time) {
      if(initial_pose_old_msg.header.stamp.toSec() == 0) {
        initial_pose_old_msg = initial_pose_new_msg;          
      }
      break;
    }
    initial_pose_old_msg = initial_pose_new_msg;
  }

  const auto initial_pose_msg = interpolatePose(initial_pose_new_msg, initial_pose_old_msg, sensor_ros_time);

  geometry_msgs::PoseWithCovarianceStamped initial_pose_cov_msg;
  initial_pose_cov_msg.header = initial_pose_msg.header;
  initial_pose_cov_msg.pose.pose = initial_pose_msg.pose;

  // geometry_msgs::TransformStamped::Ptr TF_map_to_base_ptr(new geometry_msgs::TransformStamped);
  // getTransform(map_frame_, "ekf_pose", TF_map_to_base_ptr, sensor_ros_time);
  // initial_pose_cov_msg.pose.pose.position.x = TF_map_to_base_ptr->transform.translation.x;
  // initial_pose_cov_msg.pose.pose.position.y = TF_map_to_base_ptr->transform.translation.y;
  // initial_pose_cov_msg.pose.pose.position.z = TF_map_to_base_ptr->transform.translation.z;
  // initial_pose_cov_msg.pose.pose.orientation = TF_map_to_base_ptr->transform.rotation;
  // 
  // generateParticle

  // align
  Eigen::Affine3d initial_pose_affine;
  Eigen::fromMsg(initial_pose_cov_msg.pose.pose, initial_pose_affine);
  const Eigen::Matrix4f initial_pose_matrix = initial_pose_affine.matrix().cast<float>();

  pcl::PointCloud<PointSource>::Ptr output_cloud(new pcl::PointCloud<PointSource>);
  const auto align_start_time = std::chrono::system_clock::now();
  ndt_ptr_->align(*output_cloud, initial_pose_matrix);
  const auto align_end_time = std::chrono::system_clock::now();
  const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() / 1000.0;
  // for (const auto& particle : particle_array) {
  //   const Eigen::Matrix4f initial_pose_matrix = tf::poseMsgToEigen(particle.initial_pose_cov_msg).matrix();
  //   ndt_ptr_->align(initial_pose_matrix);
  // }

  // selectBestParticle
  const Eigen::Matrix4f result_pose_matrix = ndt_ptr_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg =  tf2::toMsg(result_pose_affine);

  const std::vector<Eigen::Matrix4f> result_pose_matrix_array = ndt_ptr_->getFinalTransformationArray();
  std::vector<geometry_msgs::Pose> result_pose_msg_array;
  for (const auto& pose_matrix : result_pose_matrix_array) {
    Eigen::Affine3d pose_affine;
    pose_affine.matrix() = pose_matrix.cast<double>();
    const geometry_msgs::Pose pose_msg =  tf2::toMsg(pose_affine);
    result_pose_msg_array.push_back(pose_msg);
  }

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

  const float transform_probability = ndt_ptr_->getTransformationProbability();

  const size_t iteration_num = ndt_ptr_->getFinalNumIteration();

  bool is_converged = true;
  if (iteration_num >= 32 || transform_probability < 4.5) {
    is_converged = false;
    std::cout << "Not Converged" << std::endl;
    std::cout << "F**********************************************************************************************K" << std::endl;
  }
  // publish
  publishTF(map_frame_, "ndt_base_link", result_pose_msg);

  pcl::PointCloud<PointSource>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<PointSource>);
  pcl::transformPointCloud(*sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);
  sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame_;
  sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);

  geometry_msgs::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  if(is_converged) {
    ndt_pose_pub_.publish(result_pose_stamped_msg);
  }

  geometry_msgs::PoseWithCovarianceStamped result_pose_with_cov_msg;
  result_pose_with_cov_msg.header.stamp = sensor_ros_time;
  result_pose_with_cov_msg.header.frame_id = map_frame_;
  result_pose_with_cov_msg.pose.pose = result_pose_msg;
  ndt_pose_with_covariance_pub_.publish(result_pose_with_cov_msg);

  initial_pose_with_covariance_pub_.publish(initial_pose_cov_msg);


  visualization_msgs::MarkerArray marker_array;
  visualization_msgs::Marker marker;
  marker.header.stamp = sensor_ros_time;
  marker.header.frame_id = map_frame_;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.scale.x = 0.3;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  int i = 0;
  marker.ns = "result_pose_matrix_array";
  marker.action = visualization_msgs::Marker::ADD;
  for (const auto& pose_msg : result_pose_msg_array)
  {
    marker.id = i++;
    marker.pose = pose_msg;
    marker.color = ExchangeColorCrc((1.0*i)/15.0);
    marker_array.markers.push_back(marker);
  }
  for ( ; i < 32;)
  {
    marker.id = i++;
    marker.pose = geometry_msgs::Pose();
    marker.color = ExchangeColorCrc(0);
    marker_array.markers.push_back(marker);
  }
  // if(!is_converged) {
    ndt_marker_pub_.publish(marker_array);
  // }
  std_msgs::Float32 exe_time_msg;
  exe_time_msg.data = exe_time;
  exe_time_pub_.publish(exe_time_msg);

  std_msgs::Float32 transform_probability_msg;
  transform_probability_msg.data = transform_probability;
  transform_probability_pub_.publish(transform_probability_msg);
  
  std_msgs::Float32 iteration_num_msg;
  iteration_num_msg.data = iteration_num;
  iteration_num_pub_.publish(iteration_num_msg);
  
  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "align_time: " << align_time << "ms" << std::endl;
  std::cout << "exe_time: " << exe_time << "ms" << std::endl;
  std::cout << "trans_prob: " << transform_probability << std::endl;
  std::cout << "iter_num: " << iteration_num << std::endl;
}

void NDTScanMatcher::callbackMapPoints(const sensor_msgs::PointCloud2::ConstPtr &map_points_msg_ptr)
{
  const auto trans_epsilon = ndt_ptr_->getTransformationEpsilon();
  const auto step_size = ndt_ptr_->getStepSize();
  const auto resolution = ndt_ptr_->getResolution();
  const auto max_iterations = ndt_ptr_->getMaximumIterations();

  std::shared_ptr<pcl::NormalDistributionsTransformModified<PointSource, PointTarget>> new_ndt_ptr_(new pcl::NormalDistributionsTransformModified<PointSource, PointTarget>);
  new_ndt_ptr_->setTransformationEpsilon(trans_epsilon);
  new_ndt_ptr_->setStepSize(step_size);
  new_ndt_ptr_->setResolution(resolution);
  new_ndt_ptr_->setMaximumIterations(max_iterations);

  pcl::PointCloud<PointTarget>::Ptr map_points_ptr(new pcl::PointCloud<PointTarget>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  new_ndt_ptr_->setInputTarget(map_points_ptr);
  // create Thread
  // detach
  pcl::PointCloud<PointSource>::Ptr output_cloud(new pcl::PointCloud<PointSource>);
  new_ndt_ptr_->align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  ndt_map_mtx_.lock();
  ndt_ptr_ = new_ndt_ptr_;
  ndt_map_mtx_.unlock();
}

void NDTScanMatcher::publishTF(const std::string &frame_id, const std::string &child_frame_id, const geometry_msgs::Pose &pose_msg) {

  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = current_scan_time_;

  transform_stamped.transform.translation.x = pose_msg.position.x;
  transform_stamped.transform.translation.y = pose_msg.position.y;
  transform_stamped.transform.translation.z = pose_msg.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}

bool NDTScanMatcher::getTransform(const std::string &target_frame, const std::string &source_frame, const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr, const ros::Time &time_stamp)
{
  if(target_frame == source_frame) {
      transform_stamped_ptr->header.stamp = time_stamp;
      transform_stamped_ptr->header.frame_id = target_frame;
      transform_stamped_ptr->child_frame_id = source_frame;
      transform_stamped_ptr->transform.translation.x = 0.0;
      transform_stamped_ptr->transform.translation.y = 0.0;
      transform_stamped_ptr->transform.translation.z = 0.0;
      transform_stamped_ptr->transform.rotation.x = 0.0;
      transform_stamped_ptr->transform.rotation.y = 0.0;
      transform_stamped_ptr->transform.rotation.z = 0.0;
      transform_stamped_ptr->transform.rotation.w = 1.0;
      return true;
  }

  try {
    *transform_stamped_ptr = tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp, ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool NDTScanMatcher::getTransform(const std::string &target_frame, const std::string &source_frame, const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr)
{
  if(target_frame == source_frame) {
      transform_stamped_ptr->header.stamp = ros::Time::now();
      transform_stamped_ptr->header.frame_id = target_frame;
      transform_stamped_ptr->child_frame_id = source_frame;
      transform_stamped_ptr->transform.translation.x = 0.0;
      transform_stamped_ptr->transform.translation.y = 0.0;
      transform_stamped_ptr->transform.translation.z = 0.0;
      transform_stamped_ptr->transform.rotation.x = 0.0;
      transform_stamped_ptr->transform.rotation.y = 0.0;
      transform_stamped_ptr->transform.rotation.z = 0.0;
      transform_stamped_ptr->transform.rotation.w = 1.0;
      return true;
  }

  try {
    *transform_stamped_ptr = tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf2::TransformException &ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

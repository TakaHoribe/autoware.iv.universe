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

#pragma once

#include <array>
#include <memory>
#include <string>
#include <deque>
#include <mutex>

#include <ros/ros.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <dynamic_reconfigure/server.h>

#include <std_msgs/Float32.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/MarkerArray.h>

#include "ndt_scan_matcher/NDTAlign.h"
// #include <pcl/registration/ndt.h>
// #include "pcl_registration/ndt.h"
#include "ndt/pcl_generic.h"
#include "ndt/pcl_modified.h"
#include "ndt/omp.h"

class NDTScanMatcher {
  using PointSource = pcl::PointXYZI;
  using PointTarget = pcl::PointXYZI;

  // TODO move file
  struct OMPParams {
    OMPParams() : search_method(pclomp::NeighborSearchMethod::KDTREE), num_threads(1) {};
    pclomp::NeighborSearchMethod search_method;
    int num_threads;
  };

  struct Particle
  {
    Particle(const geometry_msgs::Pose& a_initial_pose, const geometry_msgs::Pose& a_result_pose, const double a_score, const int a_iteration)
      : initial_pose(a_initial_pose)
      , result_pose(a_result_pose)
      , score(a_score)
      , iteration(a_iteration)
      {};
    geometry_msgs::Pose initial_pose;
    geometry_msgs::Pose result_pose;
    double score;
    int iteration;
  };

  enum class NDTImplementType {
     PCL_GENERIC = 0
   , PCL_MODIFIED = 1
   , OMP = 2
  };

public:
  NDTScanMatcher(ros::NodeHandle nh, ros::NodeHandle private_nh);
  ~NDTScanMatcher();

private:

  /**
   * @brief A callback of dynamic_reconfigure that sets ndt_slam parameters.
   * @fn configCallback
   * @param[in] config ndt_slam config received from dynamic_reconfigure.
   * @param[in] level unused
   */
  // void configCallback(const ndt_slam::NDTScanMatcherConfig &config, uint32_t level);

  bool serviceNDTAlign(ndt_scan_matcher::NDTAlign::Request &req, ndt_scan_matcher::NDTAlign::Response &res);

  void callbackMapPoints(const sensor_msgs::PointCloud2::ConstPtr &pointcloud2_msg_ptr);
  void callbackSensorPoints(const sensor_msgs::PointCloud2::ConstPtr &pointcloud2_msg_ptr);
  void callbackInitialPose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &pose_conv_msg_ptr);

  geometry_msgs::PoseWithCovarianceStamped alignUsingMonteCarlo(const std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> &ndt_ptr, const geometry_msgs::PoseWithCovarianceStamped &initial_pose_with_cov);

  void updateTransforms();

  void publishTF(const std::string &frame_id, const std::string &child_frame_id, const geometry_msgs::Pose &pose_msg);
  bool getTransform(const std::string &target_frame, const std::string &source_frame, const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr, const ros::Time &time_stamp);
  bool getTransform(const std::string &target_frame, const std::string &source_frame, const geometry_msgs::TransformStamped::Ptr &transform_stamped_ptr);

  void publishMarkerForDebug(const Particle &particle_array, const size_t i);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  ros::Subscriber initial_pose_sub_;
  ros::Subscriber map_points_sub_;
  ros::Subscriber sensor_points_sub_;

  ros::Publisher sensor_aligned_pose_pub_;
  ros::Publisher ndt_pose_pub_;
  ros::Publisher ndt_pose_with_covariance_pub_;
  ros::Publisher initial_pose_with_covariance_pub_;
  ros::Publisher exe_time_pub_;
  ros::Publisher transform_probability_pub_;
  ros::Publisher iteration_num_pub_;
  ros::Publisher initial_to_result_distance_pub_;
  ros::Publisher ndt_marker_pub_;
  ros::Publisher ndt_monte_colro_initial_pose_marker_pub_;

  // dynamic_reconfigure::Server<ndt_slam::NDTScanMatcherConfig> server_;
  // dynamic_reconfigure::Server<ndt_slam::NDTScanMatcherConfig>::CallbackType f_;

  ros::ServiceServer service_;

  tf2_ros::Buffer tf2_buffer_;
  tf2_ros::TransformListener tf2_listener_;
  tf2_ros::TransformBroadcaster tf2_broadcaster_;

  NDTImplementType ndt_implement_type_;
  std::shared_ptr<NormalDistributionsTransformBase<PointSource, PointTarget>> ndt_ptr_;

  Eigen::Matrix4f base_to_sensor_matrix_;
  std::string base_frame_;
  std::string map_frame_;
  double converged_param_transform_probability_;

  std::deque<boost::shared_ptr<const geometry_msgs::PoseWithCovarianceStamped>> initial_pose_msg_ptr_array_;
  ros::Time current_scan_time_;
  std::mutex ndt_map_mtx_;

  OMPParams omp_params_;
};

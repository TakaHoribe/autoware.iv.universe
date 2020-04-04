/*
 * Copyright 2020 Autoware Foundation. All rights reserved.
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

#include "lidar_apollo_instance_segmentation/detector.h"
#include "lidar_apollo_instance_segmentation/feature_map.h"
#include <boost/filesystem.hpp>

LidarApolloInstanceSegmentation::LidarApolloInstanceSegmentation() : nh_(""), pnh_("~"){
  int range, width, height;
  bool use_intensity_feature, use_constant_feature;
  std::string engine_file;
  pnh_.param<float>("score_threshold", score_threshold_, 0.8);
  pnh_.param<int>("range", range, 60);
  pnh_.param<int>("width", width, 640);
  pnh_.param<int>("height", height, 640);
  pnh_.param("engine_file", engine_file, std::string("vls-128.engine"));
  pnh_.param<bool>("use_intensity_feature", use_intensity_feature, true);
  pnh_.param<bool>("use_constant_feature", use_constant_feature, true);

  // load weight file
  std::ifstream fs(engine_file);
  if (fs.is_open()) {
    net_ptr_.reset(new Tn::trtNet(engine_file));
  } else {
    ROS_ERROR("Could not find %s.", engine_file.c_str());
  }

  cluster2d_.reset(new Cluster2D());
  if (!cluster2d_->init(height, width, range)) {
    ROS_ERROR("Fail to Initialize cluster2d");
  }

  // feature map generator
  feature_generator_ = std::make_shared<FeatureGenerator>(width, height, range, use_constant_feature, use_intensity_feature);
}

bool LidarApolloInstanceSegmentation::detectDynamicObjects(
    const sensor_msgs::PointCloud2& input, autoware_perception_msgs::DynamicObjectWithFeatureArray& output) {

  // convert from ros to pcl
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pointcloud_raw_ptr(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(input, *pcl_pointcloud_raw_ptr);

  // generate feature map
  std::shared_ptr<FeatureMapInterface> feature_map_ptr = feature_generator_->generate(pcl_pointcloud_raw_ptr);

  // inference
  std::unique_ptr<float[]> inferred_data(new float[net_ptr_->getOutputSize() / sizeof(float)]);
  net_ptr_->doInference(feature_map_ptr->map_data.data(), inferred_data.get());


  float *inferred_data_ptr = inferred_data.get();

  const float objectness_thresh = 0.5;

  pcl::PointIndices valid_idx;
  auto &indices = valid_idx.indices;
  indices.resize(pcl_pointcloud_raw_ptr->size());
  std::iota(indices.begin(), indices.end(), 0);
  cluster2d_->cluster(inferred_data_ptr, pcl_pointcloud_raw_ptr, valid_idx, objectness_thresh,
                      true /*use all grids for clustering*/);
  cluster2d_->filter(inferred_data_ptr);
  cluster2d_->classify(inferred_data_ptr);

  float height_thresh = 0.5;
  int min_pts_num = 3;

  cluster2d_->getObjects(score_threshold_, height_thresh, min_pts_num, output,
                         input.header);
  output.header = input.header;
  return true;
}
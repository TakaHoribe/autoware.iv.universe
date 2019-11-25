/*
 *  Copyright (c) 2018, TierIV, Inc
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  * Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 *  * Neither the name of Autoware nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <pluginlib/class_list_macros.h>
#include "pointcloud_preprocessor/outlier_filter/voxel_grid_outlier_filter_nodelet.h"


#include <pcl/segmentation/segment_differences.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pointcloud_preprocessor
{
bool VoxelGridOutlierFilterNodelet::child_init(ros::NodeHandle &nh, bool &has_service)
{
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<dynamic_reconfigure::Server<pointcloud_preprocessor::VoxelGridOutlierFilterConfig> >(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::VoxelGridOutlierFilterConfig>::CallbackType f = boost::bind(&VoxelGridOutlierFilterNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void VoxelGridOutlierFilterNodelet::filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices,
                                     PointCloud2 &output)
{
  boost::mutex::scoped_lock lock(mutex_);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_input(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_output(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input, *pcl_input);
  pcl_output->points.reserve(pcl_input->points.size());
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(pcl_input);
  filter.setLeafSize(voxel_size_x_, voxel_size_y_, voxel_size_z_);
  filter.setMinimumPointsNumberPerVoxel(voxel_points_threshold_);
  filter.filter(*pcl_output);

  pcl::toROSMsg(*pcl_output, output);
  output.header = input->header;
}

void VoxelGridOutlierFilterNodelet::subscribe()
{
  Filter::subscribe();
}

void VoxelGridOutlierFilterNodelet::unsubscribe()
{
  Filter::unsubscribe();
}

void VoxelGridOutlierFilterNodelet::config_callback(pointcloud_preprocessor::VoxelGridOutlierFilterConfig &config, uint32_t level)
{
  boost::mutex::scoped_lock lock(mutex_);

  if (voxel_size_x_ != config.voxel_size_x)
  {
    voxel_size_x_ = config.voxel_size_x;
    NODELET_DEBUG("[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(), config.voxel_size_x);
  }
  if (voxel_size_y_ != config.voxel_size_y)
  {
    voxel_size_y_ = config.voxel_size_y;
    NODELET_DEBUG("[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(), config.voxel_size_y);
  }
  if (voxel_size_z_ != config.voxel_size_z)
  {
    voxel_size_z_ = config.voxel_size_z;
    NODELET_DEBUG("[%s::config_callback] Setting new distance threshold to: %f.", getName().c_str(), config.voxel_size_z);
  }
  if (voxel_points_threshold_ != config.voxel_points_threshold)
  {
    voxel_points_threshold_ = config.voxel_points_threshold;
    NODELET_DEBUG("[%s::config_callback] Setting new distance threshold to: %d.", getName().c_str(), config.voxel_points_threshold);
  }
  // ---[ These really shouldn't be here, and as soon as dynamic_reconfigure improves, we'll remove them and inherit from Filter
  if (tf_input_frame_ != config.input_frame)
  {
    tf_input_frame_ = config.input_frame;
    NODELET_DEBUG ("[config_callback] Setting the input TF frame to: %s.", tf_input_frame_.c_str ());
  }
  if (tf_output_frame_ != config.output_frame)
  {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG("[config_callback] Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }
  // ]---
}

} // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::VoxelGridOutlierFilterNodelet, nodelet::Nodelet);
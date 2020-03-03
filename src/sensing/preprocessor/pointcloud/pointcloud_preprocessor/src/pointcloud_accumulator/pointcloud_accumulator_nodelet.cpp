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

#include "pointcloud_preprocessor/pointcloud_accumulator/pointcloud_accumulator_nodelet.h"
#include <pluginlib/class_list_macros.h>

namespace pointcloud_preprocessor {
bool PointcloudAccumulatorNodelet::child_init(ros::NodeHandle& nh, bool& has_service) {
  // Enable the dynamic reconfigure service
  has_service = true;
  srv_ = boost::make_shared<dynamic_reconfigure::Server<pointcloud_preprocessor::PointcloudAccumulatorConfig> >(nh);
  dynamic_reconfigure::Server<pointcloud_preprocessor::PointcloudAccumulatorConfig>::CallbackType f =
      boost::bind(&PointcloudAccumulatorNodelet::config_callback, this, _1, _2);
  srv_->setCallback(f);
  return (true);
}

void PointcloudAccumulatorNodelet::filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices,
                                          PointCloud2& output) {
  boost::mutex::scoped_lock lock(mutex_);
  pointcloud_buffer_.push_front(input);
  ros::Time last_time = input->header.stamp;
  pcl::PointCloud<pcl::PointXYZ> pcl_input;
  pcl::PointCloud<pcl::PointXYZ> pcl_output;
  for (size_t i = 0; i < pointcloud_buffer_.size(); i++) {
    if (accumulation_time_sec_ < (last_time - pointcloud_buffer_.at(i)->header.stamp).toSec()) break;
    pcl::fromROSMsg(*pointcloud_buffer_.at(i), pcl_input);
    pcl_output += pcl_input;
  }
  pcl::toROSMsg(pcl_output, output);
  output.header = input->header;
}

void PointcloudAccumulatorNodelet::subscribe() { Filter::subscribe(); }

void PointcloudAccumulatorNodelet::unsubscribe() { Filter::unsubscribe(); }

void PointcloudAccumulatorNodelet::config_callback(pointcloud_preprocessor::PointcloudAccumulatorConfig& config,
                                                   uint32_t level) {
  boost::mutex::scoped_lock lock(mutex_);

  if (accumulation_time_sec_ != config.accumulation_time_sec) {
    accumulation_time_sec_ = config.accumulation_time_sec;
    NODELET_DEBUG("[%s::config_callback] Setting new accumulation time to: %f.", getName().c_str(),
                  config.accumulation_time_sec);
  }
  if (pointcloud_buffer_.size() != (size_t)config.pointcloud_buffer_size) {
    NODELET_DEBUG("[%s::config_callback] Setting new buffer size to: %d.", getName().c_str(),
                  config.pointcloud_buffer_size);
  }
  pointcloud_buffer_.set_capacity((size_t)config.pointcloud_buffer_size);
  // ---[ These really shouldn't be here, and as soon as dynamic_reconfigure improves, we'll remove them and inherit
  // from Filter
  if (tf_input_frame_ != config.input_frame) {
    tf_input_frame_ = config.input_frame;
    NODELET_DEBUG("[config_callback] Setting the input TF frame to: %s.", tf_input_frame_.c_str());
  }
  if (tf_output_frame_ != config.output_frame) {
    tf_output_frame_ = config.output_frame;
    NODELET_DEBUG("[config_callback] Setting the output TF frame to: %s.", tf_output_frame_.c_str());
  }
  // ]---
}

}  // namespace pointcloud_preprocessor

PLUGINLIB_EXPORT_CLASS(pointcloud_preprocessor::PointcloudAccumulatorNodelet, nodelet::Nodelet);
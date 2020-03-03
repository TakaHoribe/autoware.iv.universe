#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>
#include "pointcloud_preprocessor/VoxelGridDownsampleFilterConfig.h"
#include "pointcloud_preprocessor/filter.h"

namespace pointcloud_preprocessor {
class VoxelGridDownsampleFilterNodelet : public pointcloud_preprocessor::Filter {
 protected:
  boost::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::VoxelGridDownsampleFilterConfig> > srv_;
  virtual void filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices, PointCloud2& output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle& nh, bool& has_service);
  void config_callback(pointcloud_preprocessor::VoxelGridDownsampleFilterConfig& config, uint32_t level);

 private:
  double voxel_size_x_;
  double voxel_size_y_;
  double voxel_size_z_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pointcloud_preprocessor

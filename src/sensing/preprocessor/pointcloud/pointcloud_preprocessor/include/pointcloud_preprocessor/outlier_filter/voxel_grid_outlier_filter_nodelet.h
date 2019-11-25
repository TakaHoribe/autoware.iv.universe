#pragma once

#include "pointcloud_preprocessor/filter.h"
#include "pointcloud_preprocessor/VoxelGridOutlierFilterConfig.h"
#include <pcl/search/pcl_search.h>
#include <pcl/filters/voxel_grid.h>

namespace pointcloud_preprocessor
{
class VoxelGridOutlierFilterNodelet : public pointcloud_preprocessor::Filter
{
protected:
  boost::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::VoxelGridOutlierFilterConfig> > srv_;
  virtual void filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle &nh, bool &has_service);
  void config_callback(pointcloud_preprocessor::VoxelGridOutlierFilterConfig &config, uint32_t level);

private:
  double voxel_size_x_;
  double voxel_size_y_;
  double voxel_size_z_;
  int voxel_points_threshold_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pointcloud_preprocessor

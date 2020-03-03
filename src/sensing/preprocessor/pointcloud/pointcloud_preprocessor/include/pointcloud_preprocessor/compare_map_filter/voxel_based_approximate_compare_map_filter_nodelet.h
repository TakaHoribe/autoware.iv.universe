#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/search/pcl_search.h>
#include "pointcloud_preprocessor/CompareMapFilterConfig.h"
#include "pointcloud_preprocessor/filter.h"

namespace pointcloud_preprocessor {
class VoxelBasedApproximateCompareMapFilterNodelet : public pointcloud_preprocessor::Filter {
 protected:
  boost::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::CompareMapFilterConfig> > srv_;
  virtual void filter(const PointCloud2::ConstPtr& input, const IndicesPtr& indices, PointCloud2& output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle& nh, bool& has_service);
  void config_callback(pointcloud_preprocessor::CompareMapFilterConfig& config, uint32_t level);
  void input_target_callback(const PointCloudConstPtr& map);

 private:
  // pcl::SegmentDifferences<pcl::PointXYZ> impl_;
  ros::Subscriber sub_map_;
  PointCloudPtr voxel_map_ptr_;
  double distance_threshold_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid_;
  bool set_map_in_voxel_grid_;

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace pointcloud_preprocessor

#pragma once

#include "pointcloud_preprocessor/filter.h"
#include "pointcloud_preprocessor/PassThroughFilterConfig.h"
#include <pcl/search/pcl_search.h>

namespace pointcloud_preprocessor
{
class PassThroughFilterNodelet : public pointcloud_preprocessor::Filter
{
protected:
  boost::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::PassThroughFilterConfig> > srv_;
  virtual void filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle &nh, bool &has_service);
  void config_callback(pointcloud_preprocessor::PassThroughFilterConfig &config, uint32_t level);

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pointcloud_preprocessor

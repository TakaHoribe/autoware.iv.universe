#pragma once

#include "pointcloud_preprocessor/filter.h"
#include "pointcloud_preprocessor/PointcloudAccumulatorConfig.h"
#include <boost/circular_buffer.hpp>

namespace pointcloud_preprocessor
{
class PointcloudAccumulatorNodelet : public pointcloud_preprocessor::Filter
{
protected:
  boost::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::PointcloudAccumulatorConfig> > srv_;
  virtual void filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle &nh, bool &has_service);
  void config_callback(pointcloud_preprocessor::PointcloudAccumulatorConfig &config, uint32_t level);

private:
  double accumulation_time_sec_;
  boost::circular_buffer<PointCloud2::ConstPtr> pointcloud_buffer_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pointcloud_preprocessor

#pragma once

#include "pointcloud_preprocessor/filter.h"
#include "pointcloud_preprocessor/RingOutlierFilterConfig.h"
#include <pcl/search/pcl_search.h>
#include <pcl/filters/voxel_grid.h>

namespace pointcloud_preprocessor
{
class RingOutlierFilterNodelet : public pointcloud_preprocessor::Filter
{
protected:
  boost::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::RingOutlierFilterConfig> > srv_;
  virtual void filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output);
  virtual void subscribe();
  virtual void unsubscribe();

  bool child_init(ros::NodeHandle &nh, bool &has_service);
  void config_callback(pointcloud_preprocessor::RingOutlierFilterConfig &config, uint32_t level);

private:
  double distance_ratio_;
  double object_length_threshold_;
  int num_points_threshold_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace pointcloud_preprocessor

namespace pcl
{
  struct PointXYZIRADT
  {
    PCL_ADD_POINT4D;
    float intensity;
    uint16_t ring;
    float azimuth;
    float distance;
    double time_stamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  } EIGEN_ALIGN16;
};

POINT_CLOUD_REGISTER_POINT_STRUCT(pcl::PointXYZIRADT,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring)
                                  (float, azimuth, azimuth)
                                  (float, distance, distance)
                                  (double, time_stamp, time_stamp))

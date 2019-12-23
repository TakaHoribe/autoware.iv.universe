#pragma once

#include <pcl/filters/crop_box.h>

#include "pointcloud_preprocessor/filter.h"
#include "pointcloud_preprocessor/CropBoxFilterConfig.h"

namespace pointcloud_preprocessor
{
class CropBoxFilterNodelet : public pointcloud_preprocessor::Filter
{
protected:
  boost::shared_ptr<dynamic_reconfigure::Server<pointcloud_preprocessor::CropBoxFilterConfig> > srv_;
  virtual void filter(const PointCloud2::ConstPtr &input, const IndicesPtr &indices, PointCloud2 &output);
  virtual void subscribe();
  virtual void unsubscribe();

  void publishCropBoxPolygon();
  bool child_init(ros::NodeHandle &nh, bool &has_service);
  void config_callback(pointcloud_preprocessor::CropBoxFilterConfig &config, uint32_t level);


private:
  /** \brief The PCL filter implementation used. */
  pcl::CropBox<pcl::PCLPointCloud2> impl_;
  ros::Publisher crop_box_polygon_pub_;
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
} // namespace pointcloud_preprocessor

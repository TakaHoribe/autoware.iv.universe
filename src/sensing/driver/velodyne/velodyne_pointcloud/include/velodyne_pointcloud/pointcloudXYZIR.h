

#ifndef __POINTCLOUDXYZIR_H
#define __POINTCLOUDXYZIR_H

#include <pcl_ros/point_cloud.h>
#include <velodyne_pointcloud/datacontainerbase.h>

#include <velodyne_pointcloud/point_types.h>

namespace velodyne_pointcloud 
{
  class PointcloudXYZIR : public velodyne_rawdata::DataContainerBase 
  {
  public:
    pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr pc;

    PointcloudXYZIR() : pc(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>) {}
  
    virtual void addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth, const float& distance, const float& intensity, const double& time_stamp) override;
  };
}
#endif //__POINTCLOUDXYZIR_H


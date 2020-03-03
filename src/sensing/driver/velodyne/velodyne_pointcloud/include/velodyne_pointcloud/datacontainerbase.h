

#ifndef __DATACONTAINERBASE_H
#define __DATACONTAINERBASE_H

#include <ros/ros.h>

namespace velodyne_rawdata {
class DataContainerBase {
 public:
  virtual void addPoint(const float& x, const float& y, const float& z, const uint16_t& ring, const uint16_t& azimuth,
                        const float& distance, const float& intensity, const double& time_stamp) = 0;
};
}  // namespace velodyne_rawdata
#endif  //__DATACONTAINERBASE_H

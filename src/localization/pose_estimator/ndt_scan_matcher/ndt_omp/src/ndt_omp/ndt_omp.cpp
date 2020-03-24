#include <ndt_omp/ndt_omp.h>
#include <ndt_omp/ndt_omp_impl.hpp>

template class ndt_omp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class ndt_omp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;

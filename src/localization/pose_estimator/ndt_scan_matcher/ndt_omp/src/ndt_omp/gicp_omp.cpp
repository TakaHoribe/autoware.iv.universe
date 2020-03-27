#include <ndt_omp/gicp_omp.h>
#include <ndt_omp/gicp_omp_impl.hpp>

template class ndt_omp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>;
template class ndt_omp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>;

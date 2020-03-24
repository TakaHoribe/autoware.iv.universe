#include <ndt_omp/voxel_grid_covariance_omp.h>
#include <ndt_omp/voxel_grid_covariance_omp_impl.hpp>

template class ndt_omp::VoxelGridCovariance<pcl::PointXYZ>;
template class ndt_omp::VoxelGridCovariance<pcl::PointXYZI>;

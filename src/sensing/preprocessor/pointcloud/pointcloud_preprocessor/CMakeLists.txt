cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_preprocessor)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++14)

set(CMAKE_BUILD_TYPE "Release")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nodelet
  pcl_ros
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_eigen
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/CompareMapFilter.cfg
                                     cfg/VoxelGridOutlierFilter.cfg
                                     cfg/RingOutlierFilter.cfg
                                     cfg/VoxelGridDownsampleFilter.cfg
                                     cfg/PointcloudAccumulator.cfg
                                     cfg/PassThroughFilter.cfg
                                     cfg/RayGroundFilter.cfg
                                     cfg/CropBoxFilter.cfg
                                     )
find_package(OpenCV REQUIRED)
find_package (Eigen3 REQUIRED)
find_package (Boost REQUIRED)
find_package (PCL REQUIRED)
find_package (OpenMP)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   sensor_msgs#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pointcloud_accumulator
  CATKIN_DEPENDS nodelet pcl_ros roscpp sensor_msgs std_msgs
  DEPENDS Boost EIGEN3 PCL
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

# Compare Map Filter
add_library(
  compare_map_filter
  src/compare_map_filter/distance_based_compare_map_filter_nodelet.cpp
  src/compare_map_filter/voxel_based_approximate_compare_map_filter_nodelet.cpp
  src/compare_map_filter/voxel_distance_based_compare_map_filter_nodelet.cpp
  src/compare_map_filter/voxel_based_compare_map_filter_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
target_link_libraries(compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(compare_map_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

add_executable(
  distance_based_compare_map_filter_node
  src/compare_map_filter/distance_based_compare_map_filter_node.cpp)
target_link_libraries(distance_based_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(distance_based_compare_map_filter_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  voxel_based_approximate_compare_map_filter_node
  src/compare_map_filter/voxel_based_approximate_compare_map_filter_node.cpp
  )
target_link_libraries(voxel_based_approximate_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(voxel_based_approximate_compare_map_filter_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  voxel_distance_based_compare_map_filter_node
  src/compare_map_filter/voxel_distance_based_compare_map_filter_node.cpp
  )
target_link_libraries(voxel_distance_based_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(voxel_distance_based_compare_map_filter_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  voxel_based_compare_map_filter_node
  src/compare_map_filter/voxel_based_compare_map_filter_node.cpp)
target_link_libraries(voxel_based_compare_map_filter_node
                      compare_map_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(voxel_based_compare_map_filter_node
                 ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# Outlier Filter
add_library(outlier_filter
            src/outlier_filter/voxel_grid_outlier_filter_nodelet.cpp
            src/outlier_filter/ring_outlier_filter_nodelet.cpp
            src/filter.cpp # copy from pcl_ros https://github.com/ros-
                           # perception/perception_pcl/issues/9
            )
target_link_libraries(outlier_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(outlier_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(voxel_grid_outlier_filter_node
               src/outlier_filter/voxel_grid_outlier_filter_node.cpp)
target_link_libraries(voxel_grid_outlier_filter_node
                      outlier_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(voxel_grid_outlier_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


add_executable(ring_outlier_filter_node
               src/outlier_filter/ring_outlier_filter_node.cpp)
target_link_libraries(ring_outlier_filter_node
                      outlier_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(ring_outlier_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# Downsample Filter
add_library(downsample_filter
            src/downsample_filter/voxel_grid_downsample_filter_nodelet.cpp
            src/filter.cpp # copy from pcl_ros https://github.com/ros-
                           # perception/perception_pcl/issues/9
            )
target_link_libraries(downsample_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(downsample_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(voxel_grid_downsample_filter_node
               src/downsample_filter/voxel_grid_downsample_filter_node.cpp)
target_link_libraries(voxel_grid_downsample_filter_node
                      downsample_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(voxel_grid_downsample_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# Pointcloud Accumulator
add_library(pointcloud_accumulator
  src/pointcloud_accumulator/pointcloud_accumulator_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
target_link_libraries(pointcloud_accumulator
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(pointcloud_accumulator ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(pointcloud_accumulator_node
  src/pointcloud_accumulator/pointcloud_accumulator_node.cpp)
target_link_libraries(pointcloud_accumulator_node
                      pointcloud_accumulator
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(pointcloud_accumulator_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# PassThrough
add_library(
  passthrough_filter
  src/passthrough_filter/passthrough_filter_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
target_link_libraries(passthrough_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(passthrough_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  passthrough_filter_node
  src/passthrough_filter/passthrough_filter_node.cpp)
target_link_libraries(passthrough_filter_node
                      passthrough_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(passthrough_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# Ray Ground Filter
add_library(ground_filter
  src/ground_filter/ray_ground_filter_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
if (OPENMP_FOUND)
  set_target_properties(ground_filter PROPERTIES
     COMPILE_FLAGS ${OpenMP_CXX_FLAGS}
     LINK_FLAGS ${OpenMP_CXX_FLAGS}
     )
endif ()
target_include_directories(ground_filter PRIVATE
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(ground_filter
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
add_dependencies(ground_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(ray_ground_filter_node
  src/ground_filter/ray_ground_filter_node.cpp)
target_link_libraries(ray_ground_filter_node
   ground_filter
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${OpenCV_LIBRARIES}
)
add_dependencies(ray_ground_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)


# CropBox
add_library(
  crop_box_filter
  src/crop_box_filter/crop_box_filter_nodelet.cpp
  src/filter.cpp # copy from pcl_ros https://github.com/ros-
                 # perception/perception_pcl/issues/9
  )
target_link_libraries(crop_box_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(crop_box_filter ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  crop_box_filter_node
  src/crop_box_filter/crop_box_filter_node.cpp)
target_link_libraries(crop_box_filter_node
                      crop_box_filter
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(crop_box_filter_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

# Concatenate data
add_library(
  concatenate_data
  src/concatenate_data/concatenate_data_nodelet.cpp
  )
target_link_libraries(concatenate_data
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(concatenate_data ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)
add_executable(
  concatenate_data_node
  src/concatenate_data/concatenate_data_node.cpp)
target_link_libraries(concatenate_data_node
                      concatenate_data
                      ${Boost_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
add_dependencies(concatenate_data_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

# Node
install(
  TARGETS
    crop_box_filter_node
    voxel_grid_downsample_filter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Nodelet
install(
  TARGETS
    compare_map_filter
    concatenate_data
    crop_box_filter
    concatenate_data
    downsample_filter
    ground_filter
    outlier_filter
    passthrough_filter
    pointcloud_accumulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    plugins
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pointcloud_accumulator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

cmake_minimum_required(VERSION 2.8.3)

project(map_file)

find_package(autoware_build_flags REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  geometry_msgs
  tf
  pcl_ros
  tf2_ros
  tf2_geometry_msgs
  autoware_msgs
  lanelet2_extension
)

find_package(PCL REQUIRED COMPONENTS io)
# See: https://github.com/ros-perception/perception_pcl/blob/lunar-devel/pcl_ros/CMakeLists.txt#L10-L22
if (NOT "${PCL_LIBRARIES}" STREQUAL "")
    # FIXME: this causes duplicates and not found error in ubuntu:zesty
    list(REMOVE_ITEM PCL_LIBRARIES "/usr/lib/libmpi.so")

    # For debian: https://github.com/ros-perception/perception_pcl/issues/139
    list(REMOVE_ITEM PCL_IO_LIBRARIES
            "vtkGUISupportQt"
            "vtkGUISupportQtOpenGL"
            "vtkGUISupportQtSQL"
            "vtkGUISupportQtWebkit"
            "vtkViewsQt"
            "vtkRenderingQt")
endif ()

find_package(CURL REQUIRED)

set(CMAKE_CXX_FLAGS "-O2 -Wall ${CMAKE_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES get_file
  CATKIN_DEPENDS
  std_msgs
  visualization_msgs
  geometry_msgs
  tf2_geometry_msgs
  autoware_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_IO_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

add_library(get_file
  lib/map_file/get_file.cpp
)
target_link_libraries(get_file ${CURL_LIBRARIES})

add_executable(pointcloud_map_loader nodes/pointcloud_map_loader/pointcloud_map_loader.cpp)
target_link_libraries(pointcloud_map_loader ${catkin_LIBRARIES} get_file ${CURL_LIBRARIES} ${PCL_IO_LIBRARIES})
add_dependencies(pointcloud_map_loader 
  ${catkin_EXPORTED_TARGETS}
)

add_executable(lanelet2_map_loader nodes/lanelet2_map_loader/lanelet2_map_loader.cpp)
target_link_libraries(lanelet2_map_loader ${catkin_LIBRARIES})
add_dependencies(lanelet2_map_loader ${catkin_EXPORTED_TARGETS})

add_executable(lanelet2_map_visualization nodes/lanelet2_map_loader/lanelet2_map_visualization.cpp)
target_link_libraries(lanelet2_map_visualization ${catkin_LIBRARIES})
add_dependencies(lanelet2_map_visualization ${catkin_EXPORTED_TARGETS})

add_executable(pointcloud_map_filter nodes/pointcloud_map_filter/pointcloud_map_filter_node.cpp nodes/pointcloud_map_filter/pointcloud_map_filter.cpp)
target_link_libraries(pointcloud_map_filter ${catkin_LIBRARIES})
add_dependencies(pointcloud_map_filter ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(TARGETS
  get_file
  pointcloud_map_loader
  lanelet2_map_loader
  lanelet2_map_visualization
  pointcloud_map_filter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
cmake_minimum_required(VERSION 3.0.2)
project(surround_obstacle_checker)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_planning_msgs
  roscpp
  sensor_msgs
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
  visualization_msgs
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  autoware_perception_msgs
    autoware_planning_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf2
    tf2_geometry_msgs
    tf2_eigen
    tf2_ros
    visualization_msgs
    pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(surround_obstacle_checker_node
  src/debug_marker.cpp
  src/node.cpp
  src/main.cpp
)
add_dependencies(surround_obstacle_checker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(surround_obstacle_checker_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

install(
  TARGETS
    surround_obstacle_checker_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    config
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

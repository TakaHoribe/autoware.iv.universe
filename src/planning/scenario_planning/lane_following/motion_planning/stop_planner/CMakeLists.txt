cmake_minimum_required(VERSION 2.8.3)
project(stop_planner)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  roscpp
  tf2
  tf2_ros
  sensor_msgs
  pcl_ros
  pcl_conversions
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_planning_msgs
    roscpp
    tf2
    tf2_ros
    sensor_msgs
    pcl_ros
    pcl_conversions
)

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)


add_executable(stop_planner
  src/stop_planner/stop_planner_core.cpp
  src/stop_planner/stop_planner_node.cpp
  src/stop_planner/obstacle_pcd_velocity_planner.cpp
  src/stop_planner/planning_utils.cpp
  src/stop_planner/visualize.cpp
  src/stop_planner/interpolate.cpp)

add_dependencies(stop_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(stop_planner ${catkin_LIBRARIES} )

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

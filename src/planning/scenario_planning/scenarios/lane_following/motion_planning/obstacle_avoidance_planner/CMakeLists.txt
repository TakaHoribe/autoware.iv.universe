cmake_minimum_required(VERSION 2.8.3)
project(obstacle_avoidance_planner)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  autoware_perception_msgs
  autoware_system_msgs
  roscpp
  tf2_ros
  osqp_interface
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
   autoware_planning_msgs
   autoware_perception_msgs
   autoware_system_msgs
   roscpp
   tf2_ros
   osqp_interface
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_compile_options(-std=c++14)


add_executable(eb_path_planner_node
  src/eb_path_planner/util.cpp
  src/eb_path_planner/spline_interpolate.cpp
  src/eb_path_planner/eb_path_smoother.cpp
  src/eb_path_planner/modify_reference_path.cpp
  src/eb_path_planner/node.cpp
  src/eb_path_planner/main.cpp)

add_dependencies(eb_path_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

target_include_directories(eb_path_planner_node
  PUBLIC
    ${Opencv_INCLUDE_DIRS}
)

target_link_libraries(eb_path_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

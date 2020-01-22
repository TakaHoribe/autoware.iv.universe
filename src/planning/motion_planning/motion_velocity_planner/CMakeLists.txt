cmake_minimum_required(VERSION 2.8.3)
project(motion_velocity_planner)


add_compile_options(-std=c++11 -Wall -O3)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_planning_msgs
  geometry_msgs
  tf2
  tf2_ros
  dynamic_reconfigure
  stop_planner
  osqp_interface
)

generate_dynamic_reconfigure_options(cfg/MotionVelocityPlanner.cfg)


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)


catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
      roscpp
      std_msgs
      autoware_planning_msgs
      geometry_msgs
      tf2
      tf2_ros
      stop_planner
      osqp_interface
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(motion_velocity_planner src/motion_velocity_planner.cpp src/motion_velocity_planner_utils.cpp src/interpolate.cpp)
add_dependencies(motion_velocity_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} motion_velocity_planner_gencfg)
target_link_libraries(motion_velocity_planner ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(motion_velocity_planner_osqp src/motion_velocity_planner_osqp.cpp src/motion_velocity_planner_utils.cpp src/interpolate.cpp)
add_dependencies(motion_velocity_planner_osqp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} motion_velocity_planner_gencfg)
target_link_libraries(motion_velocity_planner_osqp ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})


cmake_minimum_required(VERSION 2.8.3)
project(velocity_planner)


add_compile_options(-std=c++11 -Wall)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_planning_msgs
  geometry_msgs
  tf2
  dynamic_reconfigure
  stop_planner
  osqp_interface
)

generate_dynamic_reconfigure_options(cfg/VelocityPlanner.cfg)


find_package(Boost REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PythonLibs REQUIRED) # for matplotlib


catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS 
      roscpp
      std_msgs
      autoware_planning_msgs
      geometry_msgs
      tf2
      stop_planner
      osqp_interface
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}  # for matplotlib
)

add_executable(velocity_planner src/velocity_planner.cpp src/velocity_planner_utils.cpp src/interpolate.cpp)
add_dependencies(velocity_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} velocity_planner_gencfg)
target_link_libraries(velocity_planner ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})

add_executable(velocity_planner_osqp src/velocity_planner_osqp.cpp src/velocity_planner_utils.cpp src/interpolate.cpp)
add_dependencies(velocity_planner_osqp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} velocity_planner_gencfg)
target_link_libraries(velocity_planner_osqp ${catkin_LIBRARIES} ${PYTHON_LIBRARIES})
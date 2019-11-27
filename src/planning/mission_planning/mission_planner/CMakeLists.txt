cmake_minimum_required(VERSION 2.8.3)
project(mission_planner)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  geometry_msgs
  lanelet2_extension
  roscpp
  tf2_ros
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS autoware_planning_msgs lanelet2_extension roscpp tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(mission_planner
  src/mission_planner_main.cpp
  src/mission_planner_core.cpp
  src/utility_functions.cpp
)
add_dependencies(mission_planner 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(mission_planner 
  ${catkin_LIBRARIES}
)

install(TARGETS mission_planner
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

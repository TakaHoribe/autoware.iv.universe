cmake_minimum_required(VERSION 2.8.3)
project(path_planner)

# add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  roscpp
  tf2_ros
  lanelet2_extension #TODO: add for prototype; should be deprecated in the future
)

catkin_package(
  INCLUDE_DIRS 
    include
  CATKIN_DEPENDS 
   autoware_planning_msgs 
   roscpp 
   tf2_ros
   lanelet2_extension
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(path_base_planner
  src/path_base_planner/node.cpp
)

target_link_libraries(path_base_planner
  ${catkin_LIBRARIES}
)

add_dependencies(path_base_planner 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS})

add_subdirectory (${CMAKE_CURRENT_SOURCE_DIR}/lib/osqp)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/lib/osqp/include)

add_executable(eb_path_planner_node
  src/eb_path_planner/horibe_interpolate.cpp
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
  # PRIVATE
  #   ${CMAKE_CURRENT_SOURCE_DIR}/lib/osqp/include
)

target_link_libraries(eb_path_planner_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  path_base_planner
  emosqpstatic
)

add_executable(path2trajectory_converter_node 
src/path2trajectory_converter/horibe_interpolate.cpp
src/path2trajectory_converter/node.cpp
src/path2trajectory_converter/main.cpp)

add_dependencies(path2trajectory_converter_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path2trajectory_converter_node
  ${catkin_LIBRARIES}
  path_base_planner
)
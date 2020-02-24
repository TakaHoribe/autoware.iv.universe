cmake_minimum_required(VERSION 2.8.3)
project(behavior_velocity_planner)

# Compile as C++14, supported in ROS Melodic and newer
add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_perception_msgs
  autoware_planning_msgs
  autoware_traffic_light_msgs
  geometry_msgs
  lanelet2_extension
  roscpp
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_eigen
  pcl_ros
  visualization_msgs
)

find_package(Eigen3 REQUIRED)

find_package(PCL REQUIRED)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_perception_msgs
    autoware_planning_msgs
    autoware_traffic_light_msgs
    geometry_msgs
    lanelet2_extension
    sensor_msgs
    tf2
    tf2_geometry_msgs
    tf2_ros
    tf2_eigen
    pcl_ros
    visualization_msgs
)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Common
add_library(scene_module_lib
  src/api.cpp
  src/data_manager.cpp
  src/utilization/path_utilization.cpp
  src/utilization/util.cpp
)

# Scene Module: Momentary Stop
add_library(scene_module_momentary_stop
  src/scene_module/momentary_stop/momentary_stop.cpp
  src/scene_module/momentary_stop/debug_marker.cpp
)

target_link_libraries(scene_module_momentary_stop
  scene_module_lib
)

# Scene Module: Crosswalk
add_library(scene_module_crosswalk
  src/scene_module/crosswalk/crosswalk.cpp
)

target_link_libraries(scene_module_crosswalk
  scene_module_lib
)

# Scene Module: Intersection
add_library(scene_module_intersection
  src/scene_module/intersection/manager.cpp
  src/scene_module/intersection/scene.cpp
  src/scene_module/intersection/debug.cpp
)

target_link_libraries(scene_module_intersection
  scene_module_lib
)

# Scene Module: Traffic Light
add_library(scene_module_traffic_light
  src/scene_module/traffic_light/traffic_light.cpp
  src/scene_module/traffic_light/debug_marker.cpp

)

target_link_libraries(scene_module_intersection
  scene_module_lib
)

# Scene Module: Blind Spot
add_library(scene_module_blind_spot
  src/scene_module/blind_spot/blind_spot.cpp
)

target_link_libraries(scene_module_intersection
  scene_module_lib
)

# Scene Module Manager
add_library(scene_module_manager
  src/planner_manager.cpp
)

target_link_libraries(scene_module_manager
  scene_module_momentary_stop
  scene_module_crosswalk
  scene_module_intersection
  scene_module_traffic_light
  scene_module_blind_spot
)

# Node
add_executable(behavior_velocity_planner_node
  src/node.cpp
  src/main.cpp
)

add_dependencies(behavior_velocity_planner_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(behavior_velocity_planner_node
  scene_module_manager
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)

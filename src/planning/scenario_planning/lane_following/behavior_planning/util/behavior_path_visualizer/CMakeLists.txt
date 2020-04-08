cmake_minimum_required(VERSION 2.8.3)
project(behavior_path_visualizer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  roscpp
  visualization_msgs
)
catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(behavior_path_visualizer_node src/behavior_path_visualizer_node.cpp src/behavior_path_visualizer_core.cpp)
add_dependencies(behavior_path_visualizer_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(behavior_path_visualizer_node ${catkin_LIBRARIES})

install(
  TARGETS
    behavior_path_visualizer_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

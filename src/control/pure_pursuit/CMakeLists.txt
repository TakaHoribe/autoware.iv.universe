cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_control_msgs
  geometry_msgs
  libplanning_utils
  roscpp
  sensor_msgs
  std_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    autoware_control_msgs
    geometry_msgs
    libplanning_utils
    sensor_msgs
    std_msgs
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit
  nodes/pure_pursuit/pure_pursuit_node.cpp
  nodes/pure_pursuit/pure_pursuit_core.cpp
  nodes/pure_pursuit/pure_pursuit_viz.cpp
)

target_link_libraries(pure_pursuit
  ${catkin_LIBRARIES}
)

add_dependencies(pure_pursuit
  ${catkin_EXPORTED_TARGETS}
)


## Install executables and/or libraries
install(TARGETS pure_pursuit
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

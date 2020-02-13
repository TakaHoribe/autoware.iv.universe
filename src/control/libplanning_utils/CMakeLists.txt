cmake_minimum_required(VERSION 2.8.3)
project(libplanning_utils)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_planning_msgs
  geometry_msgs
  roscpp
  tf2
  tf2_eigen
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    planning_utils
  CATKIN_DEPENDS
    autoware_planning_msgs
    geometry_msgs
    tf2
    tf2_eigen
)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


add_library(planning_utils
  src/planning_utils.cpp
  src/pure_pursuit.cpp
  src/interpolate.cpp
)

target_link_libraries(planning_utils ${catkin_LIBRARIES})

add_dependencies(planning_utils ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(TARGETS planning_utils
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

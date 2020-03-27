cmake_minimum_required(VERSION 2.8.3)
project(motion_path_planner_scene_test)

add_compile_options(-std=c++14 -O3)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  INCLUDE_DIRS
  CATKIN_DEPENDS
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

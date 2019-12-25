cmake_minimum_required(VERSION 2.8.3)
project(planning_utils)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rostest
        rosunit
        autoware_planning_msgs
        geometry_msgs
        tf2
)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp
        LIBRARIES planning_utils
#  DEPENDS system_lib
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(planning_utils src/planning_utils.cpp)

target_link_libraries(planning_utils
        ${catkin_LIBRARIES}
        ${CMAKE_DL_LIBS}
)

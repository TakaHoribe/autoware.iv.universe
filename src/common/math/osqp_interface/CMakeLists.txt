cmake_minimum_required(VERSION 2.8.3)
project(osqp_interface)

add_compile_options(-std=c++11 -Ofast)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rostest
        rosunit
)
find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include lib/osqp-0.5.0-linux64/include
        # INCLUDE_DIRS include lib/osqp-0.6.0.dev20-linux64/include
        LIBRARIES osqp_interface
        CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories(
        include
        lib/osqp-0.5.0-linux64/include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_library(osqp_interface src/osqp_interface.cpp)

target_link_libraries(osqp_interface
        ${catkin_LIBRARIES}
        ${CMAKE_CURRENT_SOURCE_DIR}/lib/osqp-0.5.0-linux64/lib/libosqp.a
        # ${CMAKE_SOURCE_DIR}/lib/osqp-0.6.0.dev20-linux64/lib/libosqp.a
        ${CMAKE_DL_LIBS}
)


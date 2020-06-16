cmake_minimum_required(VERSION 3.0.2)
project(milee_description)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY
    launch
    mesh
    urdf
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

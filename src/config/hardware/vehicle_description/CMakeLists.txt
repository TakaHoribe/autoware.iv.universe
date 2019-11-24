cmake_minimum_required(VERSION 2.8.3)
project(vehicle_description)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
xacro
)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY mesh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

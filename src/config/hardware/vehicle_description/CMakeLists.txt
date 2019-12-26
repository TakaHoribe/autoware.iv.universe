cmake_minimum_required(VERSION 2.8.3)
project(vehicle_description)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
  roscpp
  xacro
)

catkin_package(
  CATKIN_DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(additional_vehicle_info_generator
  src/additional_vehicle_info_generator_node.cpp
)

add_dependencies(additional_vehicle_info_generator
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(additional_vehicle_info_generator
  ${catkin_LIBRARIES}
)

install(DIRECTORY mesh
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

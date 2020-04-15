cmake_minimum_required(VERSION 2.8.3)
project(accel_map_converter)
add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  autoware_vehicle_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES accel_map_converter
  CATKIN_DEPENDS roscpp std_msgs autoware_vehicle_msgs geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(accel_map_converter
  src/accel_map.cpp
  src/brake_map.cpp
  src/csv_loader.cpp
  src/interpolate.cpp
)
target_link_libraries(accel_map_converter ${catkin_LIBRARIES})
add_dependencies(accel_map_converter ${catkin_EXPORTED_TARGETS})

add_executable(accel_map_converter_node
  src/node.cpp
  src/main.cpp
)
add_dependencies(accel_map_converter_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(accel_map_converter_node ${catkin_LIBRARIES} accel_map_converter)

install(TARGETS
  accel_map_converter
  accel_map_converter_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
    data
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

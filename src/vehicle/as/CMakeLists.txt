cmake_minimum_required(VERSION 2.8.3)
project(as)
add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED COMPONENTS
  roscpp
  message_filters
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  pacmod_msgs
  autoware_control_msgs
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
  message_filters
  std_msgs
  geometry_msgs
  automotive_platform_msgs
  automotive_navigation_msgs
  pacmod_msgs
  autoware_control_msgs
)

include_directories(
  include/lexus_converter
  ${catkin_INCLUDE_DIRS}
)

add_executable(ssc_interface
  nodes/ssc_interface/ssc_interface_node.cpp
  nodes/ssc_interface/ssc_interface.cpp
)

add_executable(lexus_converter
  nodes/lexus_converter/lexus_converter.cpp
  nodes/lexus_converter/lexus_converter_node.cpp
  nodes/lexus_converter/accel_map.cpp
  nodes/lexus_converter/brake_map.cpp
  nodes/lexus_converter/csv.cpp
  nodes/lexus_converter/interpolate.cpp
  )

add_executable(test_lexus_converter
  nodes/lexus_converter/test_lexus_converter.cpp
  nodes/lexus_converter/lexus_converter.cpp
  nodes/lexus_converter/accel_map.cpp
  nodes/lexus_converter/brake_map.cpp
  nodes/lexus_converter/csv.cpp
  nodes/lexus_converter/interpolate.cpp
  )

add_executable(pacmod_sim_interface nodes/pacmod_sim_interface/pacmod_sim_interface.cpp)

target_link_libraries(ssc_interface ${catkin_LIBRARIES})
target_link_libraries(lexus_converter ${catkin_LIBRARIES})
target_link_libraries(test_lexus_converter ${catkin_LIBRARIES})
target_link_libraries(pacmod_sim_interface ${catkin_LIBRARIES})

add_dependencies(ssc_interface ${catkin_EXPORTED_TARGETS})
add_dependencies(lexus_converter ${catkin_EXPORTED_TARGETS})
add_dependencies(test_lexus_converter ${catkin_EXPORTED_TARGETS})
add_dependencies(pacmod_sim_interface ${catkin_EXPORTED_TARGETS})

install(TARGETS ssc_interface
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS lexus_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  PATTERN ".svn" EXCLUDE
)
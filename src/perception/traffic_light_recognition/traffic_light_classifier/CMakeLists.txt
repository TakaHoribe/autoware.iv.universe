cmake_minimum_required(VERSION 2.8.3)
project(traffic_light_classifier)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  autoware_traffic_light_msgs
  cv_bridge
  roscpp
  sensor_msgs
  image_transport
  message_filters
  dynamic_reconfigure
)

generate_dynamic_reconfigure_options(cfg/HSVFilter.cfg)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    autoware_traffic_light_msgs
    cv_bridge
    roscpp
    sensor_msgs
    image_transport
    message_filters
    dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(traffic_light_classifier_node
  src/color_classifier.cpp
  src/node.cpp
  src/main.cpp
)

add_dependencies(traffic_light_classifier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(traffic_light_classifier_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

#############
## Install ##
#############

install(
  TARGETS
    traffic_light_classifier_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(
  DIRECTORY
    launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

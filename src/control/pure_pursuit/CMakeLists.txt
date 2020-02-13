cmake_minimum_required(VERSION 2.8.3)
project(pure_pursuit)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        std_msgs
        tf
        geometry_msgs
        autoware_msgs
        autoware_config_msgs
        pcl_ros
        pcl_conversions
        sensor_msgs
        tablet_socket_msgs
        autoware_health_checker
        amathutils_lib
        libplanning_utils
        autoware_planner_msgs
        )

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS roscpp
        std_msgs
        tf
        geometry_msgs
        autoware_msgs
        autoware_config_msgs
        pcl_ros
        pcl_conversions
        sensor_msgs
        tablet_socket_msgs
        autoware_health_checker
        amathutils_lib
        libplanning_utils
        autoware_planner_msgs
)

###########
## Build ##
###########

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall -Wextra -Werror ${CMAKE_CXX_FLAGS}")

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

add_executable(pure_pursuit_rev2
        nodes/pure_pursuit/pure_pursuit_node.cpp
        nodes/pure_pursuit/pure_pursuit_core.cpp
        nodes/pure_pursuit/pure_pursuit_viz.cpp
        )
target_link_libraries(pure_pursuit_rev2 ${catkin_LIBRARIES})
add_dependencies(pure_pursuit_rev2
        ${catkin_EXPORTED_TARGETS})


## Install executables and/or libraries
install(TARGETS pure_pursuit_rev2
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(test-pure_pursuit_core
            test/test_pure_pursuit_core.test
            test/src/test_pure_pursuit_core.cpp
            nodes/pure_pursuit/pure_pursuit_core.cpp
            nodes/pure_pursuit/pure_pursuit_viz.cpp
            )
    add_dependencies(test-pure_pursuit_core ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-pure_pursuit_core ${catkin_LIBRARIES})

    add_rostest_gtest(test-pure_pursuit_viz
            test/test_pure_pursuit_viz.test
            test/src/test_pure_pursuit_viz.cpp
            )
    add_dependencies(test-pure_pursuit_viz ${catkin_EXPORTED_TARGETS})
    target_link_libraries(test-pure_pursuit_viz ${catkin_LIBRARIES})
endif ()
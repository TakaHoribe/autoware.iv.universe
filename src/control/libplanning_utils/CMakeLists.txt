cmake_minimum_required(VERSION 2.8.3)
project(libplanning_utils)
add_compile_options(-std=c++11 -O2 -Wall -Wextra -Werror)

find_package(catkin REQUIRED COMPONENTS
        autoware_build_flags
        roscpp
        geometry_msgs
        autoware_planner_msgs
        autoware_msgs
        tf2
        tf2_eigen
        )

find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES planning_utils
        CATKIN_DEPENDS roscpp geometry_msgs autoware_planner_msgs autoware_msgs tf2 tf2_eigen
)

###########
## Build ##
###########


include_directories(
        include
        ${EIGEN3_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)


add_library(planning_utils
        src/planning_utils.cpp
        src/pure_pursuit.cpp
        src/interpolate.cpp
        )

target_link_libraries(planning_utils ${catkin_LIBRARIES})
add_dependencies(planning_utils ${catkin_EXPORTED_TARGETS})

## Install executables and/or libraries
install(TARGETS planning_utils
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

add_rostest_gtest(test-libplanning_utils test/test_libplanning_utils.test test/src/test_libplanning_utils.cpp src/planning_utils.cpp)
add_dependencies(test-libplanning_utils ${catkin_EXPORTED_TARGETS})
target_link_libraries(test-libplanning_utils ${catkin_LIBRARIES})

add_rostest_gtest(test-pure_pursuit
        test/test_pure_pursuit.test
        test/src/test_pure_pursuit.cpp
        src/pure_pursuit.cpp
        src/planning_utils.cpp
        )
add_dependencies(test-pure_pursuit ${catkin_EXPORTED_TARGETS})
target_link_libraries(test-pure_pursuit ${catkin_LIBRARIES})
endif ()
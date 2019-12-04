cmake_minimum_required(VERSION 2.8.3)
project(ndt_slam)

add_compile_options(-std=c++14)

find_package(PCL REQUIRED)


#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    add_definitions(-DOPENMP_FOUND)
#endif ()
#
#find_package(CUDA)
#
#if (CUDA_FOUND)
#    add_definitions(-DCUDA_FOUND)
#endif ()

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

find_package(catkin REQUIRED COMPONENTS
        roscpp
        message_filters
        tf2
        tf2_ros
        tf2_sensor_msgs
        tf2_geometry_msgs
        std_msgs
        nav_msgs
        sensor_msgs
        pcl_conversions
        # velodyne_pointcloud
        pcl_ros
        dynamic_reconfigure
        )

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(config/ndt_slam.cfg)

###################################
## catkin specific configuration ##
###################################
catkin_package(
        CATKIN_DEPENDS std_msgs
                       tf2_ros
                       tf2_sensor_msgs
                       tf2_geometry_msgs
        DEPENDS PCL
)

###########
## Build ##
###########

include_directories(include ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS})

SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(ndt_slam
        src/ndt_slam_node.cpp
        src/ndt_slam_core.cpp
        src/data_structs.h
        src/pose_linear_interpolator.h
        src/pose_linear_interpolator.cpp
        src/matching_score.h
        src/matching_score.cpp
        src/map_manager.h
        src/map_manager.cpp
        src/util_functions.h
        src/util_functions.cpp
        src/convert_ros_msgs.h
        src/convert_ros_msgs.cpp
        src/lidar_localizer.h
        src/lidar_localizer.cpp
        src/ndt_slam_base.h
        src/ndt_slam_base.cpp
        src/ndt_slam_pcl.h
        src/ndt_slam_pcl.cpp
        src/pcl_registration/ndt.hpp
        src/pcl_registration/ndt.h
        src/pcl_registration/ndt.cpp
)

#if(CUDA_FOUND)
#target_include_directories(ndt_slam PRIVATE ${CUDA_INCLUDE_DIRS})
#endif()

# make sure configure headers are build before node using them
add_dependencies(ndt_slam ndt_slam_gencfg)

target_link_libraries(ndt_slam ${catkin_LIBRARIES})

install(TARGETS
            ndt_slam
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE)

if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)

    add_rostest_gtest(test_launch_ndt_slam
            test/test_launch_ndt_slam.test
            test/src/test_launch_ndt_slam.cpp
            )

    target_link_libraries(test_launch_ndt_slam
            ${catkin_LIBRARIES}
            )
endif ()

cmake_minimum_required(VERSION 2.8.3)
project(lidar_apollo_cnn_seg_detect)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
        # autoware_build_flags
        roscpp
        tf
        tf_conversions
        pcl_ros
        sensor_msgs
        geometry_msgs
        autoware_perception_msgs
        )

find_package(CUDA)
find_package(OpenCV REQUIRED)

catkin_package(
        INCLUDE_DIRS
        include
        CATKIN_DEPENDS
        roscpp
        tf
        tf_conversions
        pcl_ros
        sensor_msgs
        geometry_msgs
        autoware_perception_msgs
)

IF (CUDA_FOUND)
    include_directories(${CUDA_INCLUDE_DIRS})

    IF ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "^arm")
        link_directories(/usr/lib/arm-linux-gnueabihf/tegra)
    endif ()
ENDIF ()

# Download caffemodel and prototxt
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
if (NOT EXISTS "${PATH}")
  execute_process(COMMAND mkdir -p ${PATH})
endif()
set(FILE "${PATH}/apollo_cnn.caffemodel")
message(STATUS "Checking and downloading apollo_cnn.caffemodel")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget --no-check-certificate "https://onedrive.live.com/download?cid=707D1E12CC298F03&resid=707D1E12CC298F03%21118847&authkey=AB3c9WU89-f5o20" -O ${PATH}/apollo_cnn.caffemodel)
endif()
set(FILE "${PATH}/apollo_cnn.prototxt")
message(STATUS "Checking and downloading apollo_cnn.prototxt")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget --no-check-certificate "https://onedrive.live.com/download?cid=707D1E12CC298F03&resid=707D1E12CC298F03%21118902&authkey=AJt9ZeldiSyMrzE" -O ${PATH}/apollo_cnn.prototxt)
endif()


###CAFFE
set(CAFFE_PATH "/usr/local/caffe")

if (EXISTS "${CAFFE_PATH}")

    include_directories(
            include
            ${catkin_INCLUDE_DIRS}
            ${OpenCV_INCLUDE_DIRS}
            ${CAFFE_PATH}/include
    )

    add_executable(lidar_apollo_cnn_seg_detect
            nodes/lidar_apollo_cnn_seg_detect_node.cpp
            nodes/cnn_segmentation.cpp
            nodes/cluster2d.cpp
            nodes/feature_generator.cpp
            )
    target_link_libraries(lidar_apollo_cnn_seg_detect
            ${catkin_LIBRARIES}
            ${OpenCV_LIBRARIES}
            ${CUDA_LIBRARIES}
            ${CUDA_CUBLAS_LIBRARIES}
            ${CUDA_curand_LIBRARY}
            ${CAFFE_PATH}/lib/libcaffe.so
            glog
            )
    add_dependencies(lidar_apollo_cnn_seg_detect
            ${catkin_EXPORTED_TARGETS}
            )

    install(TARGETS
            lidar_apollo_cnn_seg_detect
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

else ()
    message("'Caffe' is not installed. 'lidar_apollo_cnn_seg_detect' will not be built.")
endif ()

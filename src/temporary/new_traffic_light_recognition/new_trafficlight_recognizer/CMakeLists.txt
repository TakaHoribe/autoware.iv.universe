cmake_minimum_required(VERSION 2.8.3)

add_compile_options(-std=c++14)

project(new_trafficlight_recognizer)
execute_process(
  COMMAND rosversion -d
  OUTPUT_VARIABLE ROS_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE
  )

set(ROS_VERSION $ENV{ROS_DISTRO})

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  roscpp
  tf
  image_transport
  nodelet
  eigen_conversions
  lanelet2_extension
  geometry_msgs
  sensor_msgs
  std_msgs
  autoware_msgs
  visualization_msgs
  autoware_traffic_light_msgs
  jsk_recognition_msgs
  )

find_package(OpenCV REQUIRED)
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

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find_package(OpenGL REQUIRED)

# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
if (CUDA_FOUND)
  message("CUDA is available!")
  message("CUDA Libs: ${CUDA_LIBRARIES}")
  message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND")
  set(CUDA_AVAIL OFF)
endif (CUDA_FOUND)

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSER_RUNTIME nvonnxparser_runtime)
find_library(NVONNXPARSER nvonnxparser)
find_library(NVINFER_PLUGIN NAMES nvinfer_plugin)
if(NVINFER AND NVPARSERS AND NVONNXPARSER AND NVONNXPARSER_RUNTIME AND NVINFER_PLUGIN)
   message("TensorRT is available!")
   message("NVINFER: ${NVINFER}")
   message("NVPARSERS: ${NVPARSERS}")
   message("NVONNXPARSER: ${NVONNXPARSER}")
   message("NVONNXPARSER_RUNTIME: ${NVONNXPARSER_RUNTIME}")
   set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
PATH_SUFFIXES lib lib64 bin
DOC "CUDNN library." )
if(CUDNN_LIBRARY)
   message("CUDNN is available!")
   message("CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
   set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()


if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  ###################################
  ## catkin specific configuration ##
  ###################################
  include_directories(
    include
    ${Eigen3_INCLUDE_DIRS}
    ${Opencv_INCLUDE_DIRS}
    ${CUDA_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tracker/libs/src/
    ${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tracker/libs/src/piotr_fhog
    ${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_tracker/libs/src/cn
    ${catkin_INCLUDE_DIRS}
    )
    
  catkin_package(
    INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS} include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    cmake_modules
    cv_bridge
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    tf
    visualization_msgs
    image_transport
    jsk_recognition_msgs
    LIBRARIES
    libutils
    libtrafficlightkcf
    libtlrdetector
    )

  set(CMAKE_CXX_FLAGS "-O2 -Wall -Wunused-variable ${CMAKE_CXX_FLAGS} -fpic -std=c++11 -pthread")

  ### utils ###
  add_library(libutils
    lib/utils.cpp
    )
  target_link_libraries(libutils
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    )
  add_dependencies(libutils
    ${catkin_EXPORTED_TARGETS}
    )


  # ### tlr_visualizer ###
  # add_executable(tlr_visualizer
  #   nodes/tlr_visualizer/visualizer_node.cpp
  #   )
  # target_link_libraries(tlr_visualizer
  #   ${catkin_LIBRARIES}
  #   ${OpenCV_LIBS}
  #   libutils
  #   )


  # ### tlr_classifier ###
  # add_executable(tlr_classifier
  #   nodes/tlr_classifier/traffic_light_classifier_node.cpp
  #   nodes/tlr_classifier/traffic_light_classifier.cpp
  #   )
  # target_link_libraries(tlr_classifier
  #   ${catkin_LIBRARIES}
  #   ${OpenCV_LIBS}
  #   libutils
  #   )


  # ### feat_proj ###
  # add_executable(feat_proj
  #   nodes/feat_proj/feat_proj.cpp
  #   )
  # target_link_libraries(feat_proj
  #   ${catkin_LIBRARIES}
  #   ${OpenCV_LIBS}
  #   )
  # add_dependencies(feat_proj
  #   ${catkin_EXPORTED_TARGETS}
  #   )

  ### feat_proj_lanelet2 ### 
  add_executable(feat_proj_lanelet2
          nodes/feat_proj_lanelet2/feat_proj_lanelet2_core.cpp
          nodes/feat_proj_lanelet2/feat_proj_lanelet2_node.cpp
          )

  target_link_libraries(feat_proj_lanelet2
          ${catkin_LIBRARIES}
          ${OpenCV_LIBS}
          )

  add_dependencies(feat_proj_lanelet2
          ${catkin_EXPORTED_TARGETS}
          )

  ### tlr_detector ###
  add_library(libtlrdetector
    nodes/tlr_detector/lib/src/EntropyCalibrator.cpp
    nodes/tlr_detector/lib/src/TrtNet.cpp
    )
    
  target_include_directories(libtlrdetector PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/nodes/tlr_detector/lib/include
  )
    
  target_link_libraries(libtlrdetector
          ${NVINFER}
          ${NVONNXPARSER}
          ${NVONNXPARSER_RUNTIME}
          ${NVINFER_PLUGIN}
          ${CUDA_LIBRARIES}
          ${CUDA_CUBLAS_LIBRARIES}
          ${CUDA_curand_LIBRARY}
          ${CUDNN_LIBRARY}
          )
  add_dependencies(libtlrdetector
    ${catkin_EXPORTED_TARGETS}
    )

  add_executable(tlr_detector
    nodes/tlr_detector/tlr_detector_node.cpp
    nodes/tlr_detector/tlr_detector.cpp
    )
  target_link_libraries(tlr_detector
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    libtlrdetector
    libutils
    )
  add_dependencies(tlr_detector
    ${catkin_EXPORTED_TARGETS}
    )


  # ### tlr_tracker ###
  # add_subdirectory(nodes/tlr_tracker/libs/src
  #   )
  # add_library(libtrafficlightkcf
  #   nodes/tlr_tracker/kcf_tracker.cpp
  #   )
  # target_link_libraries(libtrafficlightkcf
  #   ${catkin_LIBRARIES}
  #   ${OpenCV_LIBRARIES}
  #   kcf
  #   )
  # add_dependencies(libtrafficlightkcf
  #   ${catkin_EXPORTED_TARGETS}
  #   )


  # ### nodelet ###
  # add_library(${PROJECT_NAME} SHARED
  #   nodes/feat_proj/feat_proj_bbox.cpp
  #   nodes/tlr_tracker/kcf_tracker_node.cpp
  #   )
  # target_link_libraries(
  #   ${PROJECT_NAME}
  #   ${catkin_LIBRARIES}
  #   ${OpenCV_LIBRARIES}
  #   ${EIGEN_LIBRARIES}
  #   libtrafficlightkcf
  #   libutils
  #   )
  endif()


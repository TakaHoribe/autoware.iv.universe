cmake_minimum_required(VERSION 2.8.3)
project(traffic_light_classifier)

add_compile_options(-std=c++14)

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

option(TRT_AVAIL "TensorRT available" OFF)
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSERS NAMES nvonnxparser)
if(NVINFER AND NVPARSERS AND NVONNXPARSERS)
  message("TensorRT is available!")
  message("NVINFER: ${NVINFER}")
  message("NVPARSERS: ${NVPARSERS}")
  message("NVONNXPARSERS: ${NVONNXPARSERS}")
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

option(CUDNN_AVAIL "CUDNN available" OFF)
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

add_library(libutils
  utils/trt_common.cpp
  )
target_link_libraries(libutils
  ${OpenCV_LIBRARIES}
  ${NVINFER}
  ${NVPARSERS}
  ${NVONNXPARSERS}
  ${CUDA_LIBRARIES}
  ${CUDA_CUBLAS_LIBRARIES}
  ${CUDA_curand_LIBRARY}
  ${CUDNN_LIBRARY}
  )
add_dependencies(libutils
  ${catkin_EXPORTED_TARGETS})

include_directories(
  include
  utils
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(traffic_light_classifier_node
  src/color_classifier.cpp
  src/cnn_classifier.cpp
  src/node.cpp
  src/main.cpp
)

add_dependencies(traffic_light_classifier_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(traffic_light_classifier_node
  libutils
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

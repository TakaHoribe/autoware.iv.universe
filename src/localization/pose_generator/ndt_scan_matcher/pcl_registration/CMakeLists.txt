cmake_minimum_required(VERSION 2.8.3)
project(pcl_registration)

add_compile_options(-std=c++14)
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(catkin REQUIRED)
find_package(PCL REQUIRED)


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(include ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(pcl_registration src/ndt.cpp
)

target_link_libraries(pcl_registration ${catkin_LIBRARIES})

install(TARGETS
            pcl_registration
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

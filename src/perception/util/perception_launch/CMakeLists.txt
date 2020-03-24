cmake_minimum_required(VERSION 2.8.3)
project(perception_launch)

find_package(catkin REQUIRED)

catkin_package()

install(FILES launch/perception.launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

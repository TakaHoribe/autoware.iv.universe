cmake_minimum_required(VERSION 2.8.3)
project(as_config)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS
)

install(DIRECTORY license/
        DESTINATION $ENV{HOME}/.ros
        PATTERN ".svn" EXCLUDE
)

install(DIRECTORY json/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/json
        PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        PATTERN ".svn" EXCLUDE
)

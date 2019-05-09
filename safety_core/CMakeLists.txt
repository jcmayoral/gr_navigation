cmake_minimum_required(VERSION 2.8.3)
project(safety_core)

find_package(catkin REQUIRED
        COMPONENTS
            safety_msgs
        )

catkin_package(
    INCLUDE_DIRS
        include
    CATKIN_DEPENDS
            safety_msgs
)


## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

cmake_minimum_required(VERSION 2.8.3)
project(gr_action_msgs)

find_package(catkin REQUIRED
        COMPONENTS
            std_msgs
            actionlib_msgs
)


add_action_files(
  DIRECTORY action
  FILES GRNavigation.action
)

generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

catkin_package(
    CATKIN_DEPENDS
        std_msgs
        actionlib_msgs
)


## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

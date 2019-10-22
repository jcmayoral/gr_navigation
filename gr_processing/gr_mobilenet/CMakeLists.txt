cmake_minimum_required(VERSION 2.8.3)
project(gr_mobilenet)
add_definitions(-std=c++11)

set(CMAKE_CXX_FLAGS "-std=c++11")

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        roscpp
        pluginlib
        safety_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
        cv_bridge
        message_filters
        tf2
        tf2_ros
        darknet_ros_msgs
)

find_package(OpenCV 3 REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES gr_safety_policies
    CATKIN_DEPENDS
        dynamic_reconfigure
        roscpp
        pluginlib
        safety_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
        cv_bridge
        message_filters
        tf2
        tf2_ros
        darknet_ros_msgs
)

file(GLOB SOURCES src/*.cpp)
add_executable(${PROJECT_NAME}_node  ${SOURCES})


add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} pcl_cuda_tools)


### INSTALLS
#install(TARGETS gr_mobilenet
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
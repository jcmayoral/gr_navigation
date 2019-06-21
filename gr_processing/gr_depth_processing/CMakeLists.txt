cmake_minimum_required(VERSION 2.8.3)
project(gr_depth_processing)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED
        COMPONENTS
        dynamic_reconfigure
        roscpp
        pluginlib
        visualization_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
        cv_bridge
        message_filters
        darknet_ros_msgs
)

find_package(OpenCV 3 REQUIRED)

#generate_dynamic_reconfigure_options(
#  cfg/ProximityPolicy.cfg
#)

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
        visualization_msgs
        sensor_msgs
        geometry_msgs
        std_msgs
        cv_bridge
        message_filters
        darknet_ros_msgs
)

file(GLOB SOURCES src/*.cpp)
add_library(${PROJECT_NAME}  ${SOURCES})


add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})


### INSTALLS
install(TARGETS gr_depth_processing
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#install(TARGETS gr_depth_processing_node
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#install(DIRECTORY config
#  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
#)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

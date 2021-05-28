cmake_minimum_required(VERSION 2.8.3)
project(gr_map_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        map_server
        roslib
        mongodb_store
        nav_msgs
        gr_action_msgs
        actionlib
        tf2
        tf2_ros
        geodesy
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        map_server
        roslib
        mongodb_store
        nav_msgs
        gr_action_msgs
        actionlib
        tf2
        tf2_ros
        geodesy
)

include_directories(include ${catkin_INCLUDE_DIRS}   ${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME}_node src/gr_map_server.cpp src/gr_map_server_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
    ${catkin_LIBRARIES}
)

## Install executables and/or libraries
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

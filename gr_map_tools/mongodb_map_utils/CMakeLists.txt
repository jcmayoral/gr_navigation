cmake_minimum_required(VERSION 2.8.3)
project(mongodb_map_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS roscpp visualization_msgs
                    geometry_msgs mongodb_store navigation_msgs
                    tf2_geometry_msgs std_msgs nav_msgs
                    gr_action_msgs geodesy actionlib)

#add_definitions(-Wno-deprecated-declarations)

catkin_package(
    CATKIN_DEPENDS
      roscpp
      visualization_msgs
      geometry_msgs
      mongodb_store
      navigation_msgs
      sensor_msgs
      tf2_geometry_msgs
      std_msgs
      gr_action_msgs
      nav_msgs
      geodesy
      actionlib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} src/mongodb_map_manager.cpp src/tf_frame_publisher.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/map_manager_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})


# Instalv
install(TARGETS ${PROJECT_NAME}_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

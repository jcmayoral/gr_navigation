cmake_minimum_required(VERSION 2.8.3)
project(gr_map_utils)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            map_server
            nav_msgs
            std_msgs
            roslib
        )

find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)

if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)


catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        map_server
        nav_msgs
        std_msgs
        roslib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(gr_map_converter src/osm_to_topological_converter.cpp)

add_executable(multi_map_server src/multi_map_server_node.cpp)
target_link_libraries(multi_map_server
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_executable(osm_map_converter src/osm_to_topological_node.cpp)
target_link_libraries(osm_map_converter 
    ${catkin_LIBRARIES}
    gr_map_converter
)


## Install executables and/or libraries
install(TARGETS multi_map_server osm_map_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
cmake_minimum_required(VERSION 2.8.3)
project(gr_map_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            map_server
            nav_msgs
            std_msgs
            roslib
            mongodb_store
            geometry_msgs
            strands_navigation_msgs
)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)


find_package(PkgConfig)
pkg_check_modules(NEW_YAMLCPP yaml-cpp>=0.5)

if(NEW_YAMLCPP_FOUND)
add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NEW_YAMLCPP_FOUND)

add_definitions(-Wno-deprecated-declarations)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        map_server
        nav_msgs
        std_msgs
        roslib
        mongodb_store
        geometry_msgs
        strands_navigation_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(multi_map_server src/multi_map_server_node.cpp)
target_link_libraries(multi_map_server
    ${catkin_LIBRARIES}
    yaml-cpp
)


add_executable(osm_map_converter src/osm_to_topological_node.cpp src/osm_to_topological_converter.cpp)
add_dependencies(osm_map_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries(osm_map_converter
    ${catkin_LIBRARIES}
)

add_executable(topological_map_converter src/topological_to_metric_node.cpp src/topological_to_metric_converter.cpp)
add_dependencies(topological_map_converter ${catkin_EXPORTED_TARGETS})
target_link_libraries(topological_map_converter
    ${catkin_LIBRARIES}
    ${Boost_LIBRARIES}
)


## Install executables and/or libraries
install(TARGETS multi_map_server osm_map_converter topological_map_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

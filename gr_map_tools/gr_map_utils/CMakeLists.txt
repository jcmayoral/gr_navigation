cmake_minimum_required(VERSION 3.0.2)
project(gr_map_utils)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        dynamic_reconfigure
        map_server
        nav_msgs
        std_msgs
        roslib
        mongodb_store
        geometry_msgs
        sensor_msgs
        std_msgs
        visualization_msgs
        geographic_msgs
        navigation_msgs
        message_generation
        tf2
        tf2_ros
        geodesy
        grid_map_ros
        grid_map_msgs
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


generate_dynamic_reconfigure_options(
  cfg/OSMMapConverter.cfg
  cfg/TopologicalMapConverter.cfg
)

add_service_files(
    FILES
    UpdateMap.srv
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        dynamic_reconfigure
        map_server
        nav_msgs
        std_msgs
        roslib
        mongodb_store
        std_msgs
        geometry_msgs
        sensor_msgs
        visualization_msgs
        geographic_msgs
        navigation_msgs
        tf2
        tf2_ros
        geodesy
        grid_map_ros
        grid_map_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS}   ${EIGEN3_INCLUDE_DIR})

add_library(gr_map_utils src/tf_frame_publisher.cpp)

add_executable(osm_to_topological src/osm_to_topological_node.cpp src/osm_to_topological_converter.cpp src/tf_frame_publisher.cpp)
add_dependencies(osm_to_topological ${catkin_EXPORTED_TARGETS} gr_map_utils_gencfg gr_map_utils_gencpp)
target_link_libraries(osm_to_topological
    ${catkin_LIBRARIES}
)

add_executable(topological_map_converter src/topological_to_metric_node.cpp src/topological_to_metric_converter.cpp  src/tf_frame_publisher.cpp)
# check why library is not working ${gr_tf_utils})

add_dependencies(topological_map_converter ${catkin_EXPORTED_TARGETS} gr_map_utils_gencfg gr_map_utils_gencpp)
target_link_libraries(topological_map_converter
    ${catkin_LIBRARIES}
)

add_executable(osm_metric_converter src/osm_to_metric_node.cpp src/osm_to_metric_converter.cpp src/tf_frame_publisher.cpp)
add_dependencies(osm_metric_converter ${catkin_EXPORTED_TARGETS} gr_map_utils_gencfg gr_map_utils_gencpp)
target_link_libraries(osm_metric_converter
    ${catkin_LIBRARIES}
)



## Install executables and/or libraries
install(TARGETS osm_to_topological topological_map_converter osm_metric_converter
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

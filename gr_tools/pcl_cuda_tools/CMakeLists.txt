cmake_minimum_required(VERSION 2.8.3)
project(pcl_cuda_tools)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED
        COMPONENTS
)

find_package(PCL)
find_package(CUDA)

find_package(Boost REQUIRED COMPONENTS
  filesystem
)




catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

#add_library(gr_map_utils src/tf_frame_publisher.cpp)

#add_executable(multi_map_server src/multi_map_server_node.cpp)
#target_link_libraries(multi_map_server
#    ${catkin_LIBRARIES}
#)


#add_dependencies(topological_map_converter ${catkin_EXPORTED_TARGETS} gr_map_utils_gencfg gr_map_utils_gencpp)

## Install executables and/or libraries
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

cmake_minimum_required(VERSION 2.8.3)
project(pcl_cuda_tools)

add_compile_options(-std=c++14)
add_definitions(-Wno-deprecated-declarations)
add_definitions(-Wno-dev)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        pcl_conversions
)

find_package(PCL 1.9 REQUIRED)
find_package(CUDA 10.0 REQUIRED)
message(STATUS ${CUDA_INCLUDE_DIRS})
message(STATUS ${CUDA_TOOLKIT_INCLUDE})



SET(CUDA_NVCC_FLAGS "-arch=sm_26" CACHE STRING "nvcc flags" FORCE)
SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
SET(LIB_TYPE STATIC)
CUDA_ADD_LIBRARY(TestLib ${LIB_TYPE} src/kinetic_ransac.cu)


find_package(Boost REQUIRED COMPONENTS
  filesystem
)


add_definitions(${PCL_DEFINITIONS})


catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
        roscpp
        pcl_conversions
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)

#add_library(gr_map_utils src/tf_frame_publisher.cpp)

add_executable(kinetic_ransac src/kinect_ransac.cpp)
target_link_libraries(kinetic_ransac
    ${catkin_LIBRARIES}
)

 target_link_libraries(kinetic_ransac
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${Boost_LIBRARIES} Threads::Threads
   TestLib
 )


#add_dependencies(topological_map_converter ${catkin_EXPORTED_TARGETS} gr_map_utils_gencfg gr_map_utils_gencpp)

## Install executables and/or libraries
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE)

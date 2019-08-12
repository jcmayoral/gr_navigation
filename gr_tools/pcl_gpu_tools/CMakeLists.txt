cmake_minimum_required(VERSION 2.8.3)
project(pcl_gpu_tools)

add_compile_options(-std=c++14)
add_definitions(-Wno-deprecated-declarations)
add_definitions(-Wno-dev)

find_package(catkin REQUIRED
        COMPONENTS
        roscpp
        pcl_conversions
        geometry_msgs
        pcl_ros
        dynamic_reconfigure
)

find_package(PCL 1.9 REQUIRED)


#SET(CUDA_NVCC_FLAGS "-arch=sm_61" CACHE STRING "nvcc flags" FORCE)
#SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
#set(CUDA_INCLUDE_DIRS /usr/local/cuda-10.1/include/ ${CUDA_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#SET(CUDA_LIBRARIES ${CUDA_LIBRARIES} "/usr/local/cuda-10.1/lib64/libcudart.so" "/usr/local/cuda-10.1/lib64/libcublas.so")

#SET(LIB_TYPE STATIC)
#CUDA_ADD_LIBRARY(TestLib ${LIB_TYPE} src/kinetic_ransac.cu)
#cuda_add_executable(
#  TestLib
#  src/kinetic_ransac.cu
#)



find_package(Boost REQUIRED COMPONENTS
  filesystem
)

generate_dynamic_reconfigure_options(
  cfg/GPUFilter.cfg
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  #${CUDA_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    pcl_conversions
    pcl_ros
    geometry_msgs
    dynamic_reconfigure
  DEPENDS
    PCL
)


add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME}
            src/gpu_example.cpp

)


#add_library(gr_map_utils src/tf_frame_publisher.cpp)
#cuda_add_library(TestLib src/kinetic_ransac.cu)
#set_property(TARGET TestLib PROPERTY CUDA_SEPARABLE_COMPILATION ON)


add_executable(${PROJECT_NAME}_node src/gpu_example_node.cpp)

target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${PROJECT_NAME}
   #${Boost_LIBRARIES}
   #Threads::Threads
   #TestLib
 )


 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${PROJECT_NAME}
   #${Boost_LIBRARIES}
   #Threads::Threads
   #TestLib
 )

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

#add_dependencies(topological_map_converter ${catkin_EXPORTED_TARGETS} gr_map_utils_gencfg gr_map_utils_gencpp)

## Install executables and/or libraries
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/cfg)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

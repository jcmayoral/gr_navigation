cmake_minimum_required(VERSION 2.8.3)

add_definitions(-Wno-deprecated-declarations)
add_definitions(-Wno-dev)

set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(CUDA REQUIRED)

#SET(CUDA_NVCC_FLAGS "-arch=sm_61" CACHE STRING "nvcc flags" FORCE)
#SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
#set(CUDA_INCLUDE_DIRS /usr/local/cuda-10.1/include/ ${CUDA_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#SET(CUDA_LIBRARIES ${CUDA_LIBRARIES} "/usr/local/cuda-10.1/lib64/libcudart.so" "/usr/local/cuda-10.1/lib64/libcublas.so")

#SET(LIB_TYPE STATIC)
cuda_add_executable(
  TestLib_executable
  src/depth_registration.cu
)

target_link_libraries(TestLib_executable
   Threads::Threads
)


#install(TARGETS TestLib_executable
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE)

cmake_minimum_required(VERSION 2.8.3)

add_definitions(-Wno-deprecated-declarations)
add_definitions(-Wno-dev)

set(CMAKE_CXX_FLAGS "-std=c++11")


find_package(CUDA REQUIRED)
#set(CUDA_SEPARABLE_COMPILATION ON)

#set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-gencode arch=compute_20,code=sm_20;--std=c++11)

#SET(CUDA_NVCC_FLAGS "-arch=sm_61" CACHE STRING "nvcc flags" FORCE)
#set(CUDA_HOST_COMPILER clang++) # shouldn't fail anymore
#SET (CUDA_VERBOSE_BUILD ON CACHE BOOL "nvcc verbose" FORCE)
#set(CUDA_INCLUDE_DIRS /usr/local/cuda-10.1/include/ ${CUDA_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
#SET(CUDA_LIBRARIES ${CUDA_LIBRARIES} "/usr/local/cuda-10.1/lib64/libcudart.so" "/usr/local/cuda-10.1/lib64/libcublas.so")

#SET(LIB_TYPE STATIC)
include_directories(include ${CUDA_INCLUDE_DIRS})


file(GLOB CUDA_FILES src/*.cu)
file(GLOB CPP_FILES src/*.cpp)


#list( APPEND CUDA_NVCC_FLAGS "-gencode arch=compute_61; -std=c++11")

cuda_add_library(
  my_cuda  STATIC ${CUDA_FILES}
)

add_executable(cuda_executable
              src/test.cpp
              src/depth_registration.cpp
              #${CUDA_FILES}
)

target_link_libraries(cuda_executable ${CUDA_LIBRARIES})


target_link_libraries(cuda_executable my_cuda
)


#install(TARGETS TestLib_executable
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

#install(DIRECTORY include/${PROJECT_NAME}
#  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#  PATTERN ".svn" EXCLUDE)

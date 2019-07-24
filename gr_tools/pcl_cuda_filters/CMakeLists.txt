set(PROJECT_NAME pcl_cuda_filters)

PROJECT(${PROJECT_NAME})
cmake_minimum_required(VERSION 2.8)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
set(CMAKE_INCLUDE_CURRENT_DIR ON)

find_package(CUDA)
find_package(PCL)

include_directories(include ${PCL_INCLUDE_DIRS})

install (DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

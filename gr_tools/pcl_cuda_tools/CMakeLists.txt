cmake_minimum_required(VERSION 2.8.3)

add_definitions(-Wno-deprecated-declarations)
add_definitions(-Wno-deprecated-gpu-targets)

add_definitions(-Wno-dev)

#set(CMAKE_CXX_FLAGS "-std=c++11")

#find_package(Eigen3)
find_package(CUDA 8 REQUIRED)
find_package(PCL 1.7.2 EXACT REQUIRED)
find_package(OpenCV 3 REQUIRED)
#SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")
SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")



include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  #${EIGEN3_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )

file(GLOB CUDA_FILES src/*.cu)

file(GLOB DEPTH_CPP_FILES src/depth*.cpp)
list(REMOVE_ITEM DEPTH_CPP_FILES "depth_test_filter.cpp")

file(GLOB FILTER_CPP_FILES src/filter*.cpp)
list(REMOVE_ITEM FILTER_CPP_FILES "filter_test.cpp")

file(GLOB CPP_FILES src/*.cpp)


file(GLOB PCL_FILES src/pcl*.cpp)



cuda_add_library(
  pcl_cuda_tools  SHARED ${CUDA_FILES} ${DEPTH_CPP_FILES} ${FILTER_CPP_FILES} ${PCL_FILES}
)

add_executable(cuda_executable_example
                src/test_depth_filter.cpp
                ${DEPTH_CPP_FILES}
)


add_executable(cuda_filter_example
              src/test_filter.cpp
              ${FILTER_CPP_FILES}
)


target_link_libraries(pcl_cuda_tools
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    #${Eigen3_LIBRARIES}
)

target_link_libraries(cuda_executable_example
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pcl_cuda_tools
)

target_link_libraries(cuda_filter_example
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
    pcl_cuda_tools
)


install(TARGETS pcl_cuda_tools
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
  DESTINATION include
  PATTERN ".svn" EXCLUDE)

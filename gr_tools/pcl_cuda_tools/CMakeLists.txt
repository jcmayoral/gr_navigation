cmake_minimum_required(VERSION 2.8.3)

#add_definitions(-Wno-deprecated-declarations)
#add_definitions(-Wno-dev)

set(CMAKE_CXX_FLAGS "-std=c++17")


find_package(CUDA REQUIRED)
find_package(PCL 1.9 REQUIRED)
find_package(OpenCV 3 REQUIRED)
SET(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS} -std=c++11")


include_directories(
  include
  ${CUDA_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set_directory_properties( PROPERTIES COMPILE_DEFINITIONS "" )


file(GLOB CUDA_FILES src/*.cu)
file(GLOB DEPTH_CPP_FILES src/depth*.cpp)
file(GLOB FILTER_CPP_FILES src/filter*.cpp)
file(GLOB CPP_FILES src/*.cpp)


cuda_add_library(
  pcl_cuda_tools  SHARED ${CUDA_FILES} ${DEPTH_CPP_FILES}
)

add_executable(cuda_executable_example
              src/depth_registration.cpp
              src/depth_test_filter.cpp
)


add_executable(cuda_filter_example
              src/filter_test.cpp
)


target_link_libraries(pcl_cuda_tools
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(cuda_executable_example
    ${CUDA_LIBRARIES}
    ${OpenCV_LIBRARIES}
    pcl_cuda_tools
)

target_link_libraries(cuda_filter_example
    ${CUDA_LIBRARIES}
    ${PCL_LIBRARIES}
)


#install(TARGETS pcl_cuda_tools cuda_executable_example cuda_filter_example
#        RUNTIME DESTINATION bin
#        LIBRARY DESTINATION lib
#        ARCHIVE DESTINATION lib)

install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/
  DESTINATION include
  PATTERN ".svn" EXCLUDE)

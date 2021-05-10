cmake_minimum_required(VERSION 2.8.3)
project(gr_line_trajectory_planner)

find_package(PCL 1.9 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  #dependencies
  roscpp
  std_msgs
  nav_msgs
  costmap_2d
  geometry_msgs
  nav_msgs
  actionlib
  actionlib_msgs
  move_base_msgs
  tf2_ros
  tf2_geometry_msgs
)

# Find SBPL from sbpl_lattice_planner
find_package(PkgConfig REQUIRED)
pkg_check_modules(SBPL REQUIRED sbpl)
find_library(SBPL_LIBRARY NAMES sbpl PATHS ${SBPL_LIBRARY_DIRS} NO_DEFAULT_PATH)
set(SBPL_LIBRARIES ${SBPL_LIBRARY})


## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${SBPL_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS include/${PROJECT_NAME}
  LIBRARIES gr_line_trajectory_planner
  CATKIN_DEPENDS roscpp std_msgs nav_msgs costmap_2d geometry_msgs nav_msgs 
                  actionlib actionlib_msgs move_base_msgs tf2_ros tf2_geometry_msgs
  DEPENDS SBPL
)


## Declare a C++ library
add_library(${PROJECT_NAME}
   src/line_trajectory_planner.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/line_planner_node.cpp src/line_trajectory_planner.cpp )

link_directories(${SBPL_LIBRARY_DIRS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
   ${SBPL_LIBRARIES}
   ${PROJECT_NAME}
)

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_node ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_gr_sbpl_trajectory_generator.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)

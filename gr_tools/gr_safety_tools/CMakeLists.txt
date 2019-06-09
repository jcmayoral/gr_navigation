cmake_minimum_required(VERSION 2.8.3)
project(gr_safety_tools)

find_package(catkin REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


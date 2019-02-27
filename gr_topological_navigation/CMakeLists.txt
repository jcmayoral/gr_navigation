cmake_minimum_required(VERSION 2.8.3)
project(gr_topological_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  strands_navigation_msgs
  topological_navigation
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    strands_navigation_msgs
    topological_navigation
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

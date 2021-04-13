cmake_minimum_required(VERSION 3.0.2)
project(gr_topological_navigation)

find_package(catkin REQUIRED COMPONENTS
  rospy
  navigation_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    navigation_msgs
    std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

cmake_minimum_required(VERSION 2.8.3)
project(gr_toponav_v2)

find_package(catkin REQUIRED COMPONENTS
  rospy
  navigation_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    rospy
    navigation_msgs
    std_msgs
)
catkin_python_setup()

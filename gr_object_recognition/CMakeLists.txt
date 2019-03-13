cmake_minimum_required(VERSION 2.8.3)
project(gr_object_recognition)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  fusion_msgs
  rospy
  dynamic_reconfigure
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES accelerometer_ros
   CATKIN_DEPENDS geometry_msgs fusion_msgs rospy dynamic_reconfigure
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

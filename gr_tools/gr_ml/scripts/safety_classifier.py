#!/usr/bin/python
import rospy
from gr_ml.keras_ros_interface import KerasWrapper

if __name__ == '__main__':
    rospy.init_node("safety_ml_node")
    kw  = KerasWrapper()
    rospy.spin()
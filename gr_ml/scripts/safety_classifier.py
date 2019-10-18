#!/usr/bin/python
import rospy
from gr_ml.pytorch_ros_interface import PyTorchWrapper

if __name__ == '__main__':
    rospy.init_node("safety_ml_node")
    kw  = PyTorchWrapper()
    rospy.spin()
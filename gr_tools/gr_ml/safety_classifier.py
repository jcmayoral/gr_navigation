import rospy
from keras_ros_interface import KerasWrapper

if '__name__' == '__name__':
    rospy.init_node("safety_ml_node")
    kw  = KerasWrapper()
    rospy.spin()
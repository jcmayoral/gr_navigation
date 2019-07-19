from sensor_msgs.msg import Image, CameraInfo
from darknet_ros_msgs.msg import BoundingBoxes
import rospy
import time


rospy.init_node("statistical_information")
objects_detected = dict()

def callback(msg):
    global objects_detected
    for bb in msg.bounding_boxes:
        if bb.Class in objects_detected:
            objects_detected[bb.Class] = objects_detected[bb.Class] + 1
        else:
            objects_detected[bb.Class] = 1


rospy.Subscriber("/darknet_ros/bounding_boxes", BoundingBoxes, callback)
rospy.spin()


print "Objects Detected ", objects_detected

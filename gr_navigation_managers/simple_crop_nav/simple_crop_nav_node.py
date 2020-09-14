#!/usr/bin/python
from simple_crop_nav import SimpleCropNavController
import rospy

from jsk_gui_msgs.msg import VoiceMessage


start_test = False

def start_cb(msg):
    print "in cb ", msg
    global start_testf
    if "start " in msg.texts:
        start_test = True

if __name__ == "__main__":
    rospy.init_node("simple_crop_nav_controller")
    #rospy.Subscriber("/Tablet/voice", VoiceMessage, start_cb)
    #while not start_test:
    #    rospy.sleep(0.1)

    msg = rospy.wait_for_message("/Tablet/voice", VoiceMessage, timeout=None)


    rospy.logerr("start test")
    controller = SimpleCropNavController()

    while not rospy.is_shutdown() and controller.is_running():
        controller.publish()
        rospy.sleep(0.1)
    rospy.loginfo("FINISHED")

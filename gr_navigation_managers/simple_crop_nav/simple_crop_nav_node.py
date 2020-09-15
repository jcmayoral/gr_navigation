#!/usr/bin/python
from simple_crop_nav import SimpleCropNavController
import rospy

if __name__ == "__main__":
    rospy.init_node("simple_crop_nav_controller")
    #rospy.Subscriber("/Tablet/voice", VoiceMessage, start_cb)
    #while not start_test:
    #    rospy.sleep(0.1)

    #msg = None

    #while msg is None:
    #    msg = rospy.wait_for_message("/Tablet/voice", VoiceMessage, timeout=None)
    #    parse_command(msg.texts)

    #print "NEXT STage"

    rospy.logerr("start test")
    controller = SimpleCropNavController()

    while not rospy.is_shutdown() and controller.is_running():
        rospy.sleep(1.0)
    #rospy.loginfo("FINISHED")

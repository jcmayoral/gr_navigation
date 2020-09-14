#!/usr/bin/python
from simple_crop_nav import SimpleCropNavController
import rospy

if __name__ == "__main__":
    rospy.init_node("simple_crop_nav_controller")
    controller = SimpleCropNavController()

    while not rospy.is_shutdown() and controller.is_running():
        controller.publish()
        rospy.sleep(0.1)
    rospy.loginfo("FINISHED")

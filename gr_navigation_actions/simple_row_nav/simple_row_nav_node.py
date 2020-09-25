#!/usr/bin/python
from simple_row_nav import SimpleRowNavController
import rospy
import sys
import os

if __name__ == "__main__":
    rospy.init_node("simple_crop_nav_controller")

    if len(sys.argv) != 2:
        print "please provide just folder name as extra args"
        sys.exit()

    folder_name = sys.argv[1]
    try:
        os.mkdir(folder_name)
    except OSError:
        print ("Creation of the directory failed")

    rospy.logerr("start test")
    controller = SimpleRowNavController(folder=folder_name)

    while not rospy.is_shutdown():# and controller.is_running():
        rospy.sleep(1.0)
    #rospy.loginfo("FINISHED")

#!/usr/bin/python
from gr_object_recognition.wheels_observers import SVMObserver
import rospy

if __name__ == '__main__':
    wheels = list()
    for i in range(4):
        SVMObserver(id=i)
        rospy.logwarn("Wheel %d observer has been initialized" % i)

    rospy.spin()

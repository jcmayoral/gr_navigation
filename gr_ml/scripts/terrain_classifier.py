#!/usr/bin/python
import rospy
from gr_ml.gr_terrain_recognition.terrain_classifier import TerrainClassifier

if __name__ == '__main__':
    tc = TerrainClassifier()
    rospy.spin()
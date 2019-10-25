#!/usr/bin/python
import rospy
import argparse
from sensor_msgs.msg import PointCloud2
from gr_ml.gr_terrain_recognition.record_detections import DetectionRecorder

help_text="This script stores detection using ros comm"


global recorder

def cb(msg):
    #global recorder
    recorder.publish_gt(msg)

if __name__ == '__main__':
    if __name__ == '__main__':
        parser = argparse.ArgumentParser(description = help_text)
        parser.add_argument("--group", "-g", default="nibio_2019")
        parser.add_argument("--topic", "-t", default="/velodyne_points")

        args = parser.parse_args()
        recorder = DetectionRecorder(folder=args.group)
        #t_subs = rospy.Subscriber(args.topic, PointCloud2, cb)

        rospy.spin()

#!/usr/bin/python
from __future__ import division
import rospy
import cv2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
from numpy import fabs,sqrt, floor
from numpy.linalg import norm
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rosbag
import argparse
import os
from PIL import Image
import warnings
import copy

help_text = 'This is a script that converts PointCloud2 message to RGB images'


class DetectionsScore:
    def __init__(self, gt_location, measured_location):
        self.index = 1
        self.gt_location = gt_location
        self.measured_location = measured_location


    def score(self, cloud_msg):
        f = open(os.path.join("results_" + self.gt_location, str(self.gt_location) + str(self.index)),"w")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--bag", "-b", help="set input bagname")
    parser.add_argument("--group", "-g", default="image")
    parser.add_argument("--topic", "-t", default="/velodyne_points")
    parser.add_argument("--meters", "-m", default=10)
    parser.add_argument("--pix_meters", "-p", default=15)
    parser.add_argument("--z_range", "-z", default=2.5)
    parser.add_argument("--debug", "-d", default=1)

    args = parser.parse_args()
    debug_mode = bool(int(args.debug))
    bag = rosbag.Bag(args.bag, mode="r")
    lidar2image = Detection(gt_location=args.group, meters = args.meters, pix_per_meter=args.pix_meters, z_range=args.z_range)
    lidar2image.save_params()

    for topic, msg, t in bag.read_messages(topics=args.topic):
        lidar2image.topic_cb(msg)
        if debug_mode:
            break
    bag.close()

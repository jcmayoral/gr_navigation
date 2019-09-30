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
    def __init__(self, gt_location = None, measured_location, mark_detections=True):
        self.index = 1
        self.gt_location = gt_location
        self.measured_location = measured_location
        self.mark_detections = mark_detections


    def score(self, cloud_msg):

        if self.gt_location is None:
            print "not gt"
        if self.mark_detections =

    def load_params(self):
        f = open(os.path.join("results_" + self.gt_location, str(self.gt_location) + str(self.index)),"r")
        f.seek(0)
        params = ast.literal_eval(f.read())
        f.close()
        print(params)
        #self.range = [params["range_min"], params["range_max"]]
        #self.meters = params["meters"]
        #self.pixels_per_meter = params["pix_per_meter"]
        self.index += 1

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--bag", "-b", help="set input bagname")
    parser.add_argument("--gt_group", "-g", default="results_nibio_2019")
    parser.add_argument("--measured_group", "-g", default="measured_nibio_2019")

    args = parser.parse_args()
    debug_mode = bool(int(args.debug))
    bag = rosbag.Bag(args.bag, mode="r")
    lidar2image = Detection(gt_location=args.gt_group, measured_location=args.measure_group)
    lidar2image.save_params()

    for topic, msg, t in bag.read_messages(topics=args.topic):
        lidar2image.topic_cb(msg)
        if debug_mode:
            break
    bag.close()

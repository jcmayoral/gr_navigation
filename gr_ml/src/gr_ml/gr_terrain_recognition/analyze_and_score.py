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
import os
from PIL import Image
import warnings
import copy
import ast
import pptk
from numpy.linalg import norm

class DetectionsScore:
    def __init__(self, gt_location = None, measured_location = None, mark_detections=True, index = 1):
        self.index = index
        self.gt_location = gt_location
        self.measured_location = measured_location
        self.mark_detections = mark_detections


    def score(self, cloud_msg):

        if self.gt_location is None or self.measured_location is None:
            print "not gt or measured location"

        raw_gt_detections = self.load_raw_values("results_", self.gt_location, split=True)
        raw_measured_detection = self.load_raw_values("measured_", self.measured_location, True, False)

        gt_detections = self.process_gt_detections(raw_gt_detections)
        measured_detections = self.process_measured_detections(raw_measured_detection)
        print measured_detections.shape, norm(measured_detections)
        print gt_detections.shape, norm(gt_detections)

        pc_points = self.extract_points(cloud_msg)
        print type(pc_points)
        acc_points = list()
        rgb_points = list()
        for i in pc_points:
            acc_points.append(i)
            rgb_points.append([0,1,1,0.01])

        for j in gt_detections:
            acc_points.append(j)
            rgb_points.append([0,0,0,1])

        for z in measured_detections:
            acc_points.append(np.append(z[:2],0))
            rgb_points.append([1,1,1,1])
        print np.asarray(acc_points).shape
        viewer = pptk.viewer(np.asarray(acc_points[:-1]), np.asarray(rgb_points[:-1]))
        viewer.set(point_size=0.25)

        raw_input("wait for key")
        viewer.close()
        self.index += 1

    def extract_points(self, pointcloud):
        gen = pc2.read_points(pointcloud, skip_nans=True, field_names=("x", "y", "z"))
        return gen

    def process_gt_detections(self, detections):
        return np.array(detections)#.reshape(-1,-1)

    def process(self,data):
        data = data.split(',')
        data = np.asarray(data[:3], dtype=np.float64)
        return data

    def process_measured_detections(self, detections):
        return np.asarray(map(self.process, detections))

    def load_raw_values(self, prefix, location, skip_header = False, split=True):
        detections = list()
        print skip_header, split
        with open(os.path.join(prefix + location, prefix + location + str(self.index)),"r") as f:
            for line in f:
                line = line.replace("[", "")
                line = line.replace("]", "")
                if skip_header:
                    skip_header = False
                    continue
                if split:
                    detections.append(np.asarray(line.split(), np.float64))
                else:
                    detections.append(line)
        #f.seek(0)
        #params = ast.literal_eval(f.read())
        #print(self.index, f.read)
        #f.close()
        #self.range = [params["range_min"], params["range_max"]]
        #self.meters = params["meters"]
        #self.pixels_per_meter = params["pix_per_meter"]
        return detections

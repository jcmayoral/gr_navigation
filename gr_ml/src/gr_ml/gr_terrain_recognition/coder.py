#!/usr/bin/python
from __future__ import division
import rospy
import cv2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image as rosImage
from safety_msgs.msg import FoundObjectsArray
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

class Features2Image:
    def __init__(self, save_image=False, ros = False, topic = "/found_object", filegroup="images", meters=5, pix_per_meter=2, z_range=5):
        self.save_image = save_image
        #TODO Add in metadata file
        self.meters = float(meters)
        self.pixels_per_meter = int(pix_per_meter)
        self.filegroup = filegroup
        self.ros = ros

        self.bridge = CvBridge()
        self.counter = 1
        #10 meters
        self.pixels_number = int(self.meters*self.pixels_per_meter)

        #assume symmetric
        #TODO FOR HIGH RANGE -> approach does not work
        self.range= [-float(z_range),float(z_range)]
        #from RGB
        self.max_value = 255

        self.size =  int(self.pixels_number*self.pixels_number)

        if not ros:
            self.create_folder()

        if ros:
            rospy.init_node("features_to_image")
            self.im_pub = rospy.Publisher("safety_terrain", rosImage, queue_size=1)
            rospy.Subscriber(topic, FoundObjectsArray, self.topic_cb, queue_size=100)
            rospy.loginfo("Node initialized")
            rospy.spin()

    def save_params(self):
        params = dict()
        params["meters"] = self.meters
        params["range_min"] = self.range[0]
        params["range_max"] = self.range[1]
        params["pix_per_meter"] = self.pixels_per_meter

        f = open(self.filegroup+"params.txt","w")
        f.seek(0)
        f.write( str(params) )
        f.close()

    def create_folder(self):
        #path = os.getcwd()
        try:
            os.mkdir(self.filegroup)
        except OSError:
            print ("Creation of the directory %s failed" % self.filegroup)
        else:
            print ("Successfully created the directory %s " % self.filegroup)

    def scalar_to_color(self,x):
        color = self.max_value/(self.range[1]-self.range[0]) * x + self.max_value/2
        return int(np.ceil(color))


    def save_image_to_file(self,img):
        name = os.path.join(self.filegroup , str(self.counter)+'.jpeg') #self.transformed_image.header.stamp
        cv2.imwrite(name,img)
        self.counter += 1

    def cv_to_ros(self, cv_img):
        try:
            ros_img = self.bridge.cv2_to_imgmsg(cv_img,"bgr8")
        except CvBridgeError, e:
            print(e)
            return
        return ros_img

    def ros_to_cv(self):
        try:
            # Convert your ROS Image message to OpenCV2
            self.transformed_image.data = np.array(self.transformed_image.data)
            cv2_img = self.bridge.imgmsg_to_cv2(self.transformed_image,"rgb8")
        except CvBridgeError, e:
            print(e)
            return
        return cv2_img

    def create_generator(self, msg):
        for i in msg.objects:
            yield i

    def topic_cb(self,msg):
        gen = self.create_generator(msg)
        self.process_features(gen)

    def calculate_mean(self,list_val):
        #warnings.filterwarnings('error')
        count = 0
        output = np.zeros((self.pixels_number, self.pixels_number   ), np.uint32)
        for i in range(self.pixels_number):
            for j in range(self.pixels_number):
                try:
                    if len(list_val[i][j][1:]) < 1:
                        continue
                    output[i,j] =np.mean(list_val[i][j][1:])
                except Warning:
                    #Default 0
                    output[i,j] = 0
                    count = count + 1
        return output

    def calculate_variance(self,list_val, mean):
        #warnings.filterwarnings('error')
        count = 0
        output = np.zeros((self.pixels_number, self.pixels_number   ), np.uint32)
        for i in range(self.pixels_number):
            for j in range(self.pixels_number):
                try:
                    if len(list_val[i][j][1:]) < 1:
                            continue
                    if len(list_val[i][j][1:]) == 1:
                        ##print "N", list_val[i][j][1:][0], mean[i,j]
                        output[i,j] = 0# int(np.sqrt(list_val[i][j][1:]))
                        continue

                    for d in list_val[i][j][1:]:
                        output[i,j] += np.power(d - mean[i,j],2)
                    output[i,j] = int(np.sqrt(output[i,j]/(len(list_val[i][j][1:])-1)))

                except Warning:
                    #Default 0
                    print "#"
                    output[i,j] = 0
                    count = count + 1
        #print np.unique(output), "IGNORING ON VARIANCE", count
        return output

    def count_elements(self,list_val):
        output = np.zeros((self.pixels_number, self.pixels_number   ), np.uint8)
        for i in range(self.pixels_number):
            for j in range(self.pixels_number):
                try:
                    output[i,j] = len(list_val[i][j][1:])
                except:
                    print "Error in count"
                    pass
        return output

    def process_features(self, features):
        rgb_color=(0, 0, 0)
        accumulator = [[list() for x in range(self.pixels_number)] for y in range(self.pixels_number)]
        
        for i in range(self.pixels_number):
            for j in range(self.pixels_number):
                accumulator[i][j].append(0)

        cvMat = np.zeros((self.pixels_number, self.pixels_number, 3), np.uint8)
        color = tuple(rgb_color)
        cvMat[:] = color

        while True:
            try:
                f = features.next()
                rospy.loginfo(f)
                x = f.centroid.point.x
                y = f.centroid.point.y
                z = f.centroid.point.z
                cell_x = int(x*self.pixels_per_meter)
                cell_y = int(y*self.pixels_per_meter)
            except:
                break

            if z > self.range[1] or z < self.range[0]:
                continue

            cell_x = int(self.pixels_number/2) + cell_x
            cell_y = int(self.pixels_number/2) + cell_y

            if cell_x > self.pixels_number-1 or cell_y > self.pixels_number-1:
                continue

            if cell_x < 0 or cell_y < 0:
                continue

            #make all z positive
            feature = (z -  self.range[0])
            featured_sigmoid = 255.0*(1. / (1. + np.exp(-feature)))

            feature_logit = np.log(featured_sigmoid/255) - np.log(1-featured_sigmoid/255)
            color_val = self.scalar_to_color(z)
            accumulator[cell_x][cell_y].append(copy.copy(color_val))

        cvMat[:,:,0] = self.count_elements(copy.copy(accumulator))
        cvMat[0,0,:] = 0
        cvMat[:,:,1] = self.calculate_mean(copy.copy(accumulator))
        cvMat[:,:,2] = self.calculate_variance(copy.copy(accumulator),copy.copy(cvMat[:,:,1]))
        
        
        if self.ros:
            ros_image = self.cv_to_ros(cvMat)
            self.im_pub.publish(ros_image)
            rospy.logerr("DONE")
        #TODO Normalize R channel
        #max_overlapping = np.max(cvMat[:,:,0])
        if self.save_image:
            self.save_image_to_file(cvMat)

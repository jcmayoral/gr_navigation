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

class Lidar2Image:
    def __init__(self, save_image=False, ros = False, filegroup="images", meters=10, pix_per_meter=10, z_range=5):
        self.save_image = save_image
        #TODO Add in metadata file
        self.meters = float(meters)
        self.pixels_per_meter = int(pix_per_meter)
        self.filegroup = filegroup

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
        self.create_folder()

        if ros:
            rospy.init_node("lidar_to_image")
            rospy.Subscriber("/velodyne_points", PointCloud2, self.topic_cb, queue_size=100)
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
        color = self.max_value/(self.range[1]-self.range[0]) * x + self.max_value/2;
        return int(np.ceil(color))


    def save_image_to_file(self,img):
        name = os.path.join(self.filegroup , str(self.counter)+'.jpeg') #self.transformed_image.header.stamp
        #file_name = os.path.join(self.filegroup , str(self.counter)+'.npy') #self.transformed_image.header.stamp

        #cv2_img.dtype='uint8'
        #rgbimg = cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_HSV2RGB)
        #params = list()
        #params.append(cv2.IMWRITE_PNG_COMPRESSION)
        #params.append(8)
        #print(img[0,0])
        #im = Image.fromarray(img)
        cv2.imwrite(name,img)

        #im.save(name, "PNG")
        #save to array
        #np.savez_compressed(name, img)
        #im = Image.open(name)
        #recovered_im = np.asarray(im)


        #recovered_im = cv2.imread(name, cv2.IMREAD_COLOR)
        #print recovered_im[0,0]


        self.counter += 1

    def ros_to_cv(self):
        try:
            # Convert your ROS Image message to OpenCV2
            #self.transformed_image.data = np.array(self.transformed_image.data)
            #self.transformed_image.data = np.round(self.transformed_image.data/self.max_value)
            self.transformed_image.data = np.array(self.transformed_image.data)

            #cv2_img = self.bridge.imgmsg_to_cv2(Image(),"mono8")
            cv2_img = self.bridge.imgmsg_to_cv2(self.transformed_image,"rgb8")
        except CvBridgeError, e:
            print(e)
            return
        return cv2_img


    def topic_cb(self,msg):
        #self.transformed_image.data = [False] * self.size * 3
        gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z"))
        self.process_pc(gen)

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
        #print np.unique(output), "IGNORING MEAN", count
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
        output = np.zeros((self.pixels_number, self.pixels_number   ), np.uint16)
        for i in range(self.pixels_number):
            for j in range(self.pixels_number):
                try:

                    #if  len(list_val[i][j][1:])>0:
                    #    print i,j, len(list_val[i][j])
                    #IGNORE 0
                    output[i,j] = len(list_val[i][j][1:])
                except:
                    #Default 0
                    print "Error in count"
                    pass
        #print np.unique(output), "COUNT"
        return output


    def process_pc(self, pc):
        #cvMat = cv2.CreateImage(self.pixels_number, self.pixels_number, cv.CV_32FC3)
        #cvMat = cv2.Mat(2,2, CV_8UC3, Scalar(0,0,255));

        rgb_color=(0, 0, 0)
        #accumulator = np.zeros((self.pixels_number, self.pixels_number), np.float32)
        accumulator = [[list() for x in range(self.pixels_number)] for y in range(self.pixels_number)]
        #accumulator = np.asarray(accumulator)
        for i in range(self.pixels_number):
            for j in range(self.pixels_number):
                accumulator[i][j].append(0)
        #    print (i,j), accumulator[i][j]


        cvMat = np.zeros((self.pixels_number, self.pixels_number, 3), np.uint8)
        color = tuple(rgb_color)
        cvMat[:] = color
        #cv.CvtColor(vis0, vis2, cv.CV_GRAY2BGR)


        for i in range(self.size-1, -1, -1):
            try:
                point = pc.next()
                x = point[0]
                y = point[1]
                z = point[2]
                cell_x = int(x*self.pixels_per_meter)
                cell_y = int(y*self.pixels_per_meter)
            except:
                continue

            if z > self.range[1] or z < self.range[0]:
                continue


            cell_x = int(self.pixels_number/2) + cell_x

            #if cell_x > self.ray_number:
            #    continue

            cell_y = int(self.pixels_number/2) + cell_y
            #if cell_y > self.ray_number:
            #    continue

            #index =  int(fabs(cell_x+ self.transformed_image.height * cell_y))
            #if self.size - index < 0:
            #    continue

            if cell_x > self.pixels_number-1 or cell_y > self.pixels_number-1:
                continue

            if cell_x < 0 or cell_y < 0:
                continue


            #make all z positive
            feature = (z -  self.range[0])
            featured_sigmoid = 255.0*(1. / (1. + np.exp(-feature)))

            feature_logit = np.log(featured_sigmoid/255) - np.log(1-featured_sigmoid/255)
            #update = min(fabs(feature), self.max_value)
            #print(z - (feature_logit + self.range[0]))
            #number of values
            color_val = self.scalar_to_color(z)
            #cvMat[cell_x,cell_y,0] +=  1
            accumulator[cell_x][cell_y].append(copy.copy(color_val))
            #cvMat[cell_x,cell_y,1] = max(cvMat[cell_x,cell_y,1],color_val)
            #cvMat[cell_x,cell_y,2] = min(cvMat[cell_x,cell_y,2],color_val)

            #TODO NOT WORKING BUT GREAT IDEA
            #cvMat[cell_x,cell_y,1] =  max(cvMat[cell_x,cell_y,1], color_val)
            #cvMat[cell_x,cell_y,2] =  min(cvMat[cell_x,cell_y,2], color_val)

        #cvMat[:,:,0] = accumulator / cvMat[:,:,0]
        cvMat[:,:,0] = self.count_elements(copy.copy(accumulator))
        cvMat[:,:,1] = self.calculate_mean(copy.copy(accumulator))
        cvMat[:,:,2] = self.calculate_variance(copy.copy(accumulator),copy.copy(cvMat[:,:,1]))

        #print np.unique(cvMat)
        #cvMat[:,:,1] = np.variance(accumulator)

        #TODO Normalize R channel
        #max_overlapping = np.max(cvMat[:,:,0])
        if self.save_image:
            self.save_image_to_file(cvMat)

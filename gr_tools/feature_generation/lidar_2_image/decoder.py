#!/usr/bin/python
import cv2
import argparse
import os
from sensor_msgs.msg import PointCloud2
import numpy as np
import pptk
import time
import copy
import pyprind
import ast
from PIL import Image
import random
import rospy
import ros_numpy

help_text = 'This is a script that converts RGB images to PointCloud2 messages'


class ImageToPc():
    def __init__(self, extension, folder, topic=None, index = -1, enable_ros= 1):
        #rospy.init_node("pointcloud_decoder")
        self.index = int(index)
        self.counter = 1
        self.folder = folder
        self.extension = extension
        self.task_done = False
        self.enable_ros = bool(int(enable_ros))
        #100 cm per meter
        if self.enable_ros:
            rospy.init_node("pointcloud_to_image")
            self.publisher = rospy.Publisher("pointcloud_conversion", PointCloud2)
            rospy.sleep(1)
            rospy.loginfo("READY")
        #self.points = list()
        #self.viewer = pptk.viewer(self.points)
        self.load_params()

    def test(self):
        print (self.points.shape)
        print (self.points[:,0].shape)
        self.rgb = np.array(self.rgb, np.float32).reshape(len(self.rgb),3)

        data = np.zeros(self.points.shape[0], dtype=[
        ('x', np.float32),
        ('y', np.float32),
        ('z', np.float32),
        ('r', np.float32),
        ('g', np.float32),
        ('b', np.float32)
        ])
        #('vectors', np.float32, (3,))])
        data['x'] = self.points[:,0]
        data['y'] = self.points[:,1]
        data['z'] = self.points[:,2]
        data['r'] = self.rgb[:,0]
        data['g'] = self.rgb[:,1]
        data['b'] = self.rgb[:,2]

        #data['vectors'] = np.arange(100)[:,np.newaxis]
        msg = ros_numpy.msgify(PointCloud2, data)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "velodyne"
        self.publisher.publish(msg)
        #ros_numpy.point_cloud2.array_to_pointcloud2(msg)

    def load_params(self):
        f = open("params.txt","r")
        f.seek(0)
        params = ast.literal_eval(f.read())
        f.close()
        print(params)
        self.range = [params["range_min"], params["range_max"]]
        self.meters = params["meters"]
        self.pixels_per_meter = params["pix_per_meter"]


    def get_next_image(self):
        print ("READING IMAGE")
        print type(self.index)
        if self.index == 0:
            img_name = os.path.join(self.folder , str(self.counter)+self.extension) #self.transformed_image.header.stamp
        else:
            img_name = os.path.join(self.folder , str(self.index)+self.extension) #self.transformed_image.header.stamp
            self.task_done = True

        try:
            #im_raw = Image.open(img_name)
            #im = np.asarray(im_raw)
            im = cv2.imread(img_name, cv2.IMREAD_COLOR)
            #im = cv2.imread(img_name)
            #im = cv2.cvtColor(im, cv2.COLOR_RGB2HSV).astype(np.float64)


            if im is None:
                print "No image", img_name
                self.task_done = True
            self.counter = self.counter + 1
            return im
        except cv2.error as e:
            print ("ERROR")
            self.task_done = True
            return None

    def compute_pc(self, img):
        height, width, channels = img.shape
        x_offset = height/2
        y_offset = width/2
        print("computing %d points ", height * width)
        pbar = pyprind.ProgBar(height*width)
        self.points = list()
        self.rgb = list()

        z_scaler =  np.fabs(self.range[1]-self.range[0])/255
        print(z_scaler)


        for i in range(height):
            for j in range(width):
                #r counter
                #g mean
                #b variance
                r,g,b = copy.copy(img[i,j,:])
                #if g >60 or b > 100:
                #    print (r,g,b)
                #    pbar.update()
                #    continue
                #print (r,g,b)
                #NOT WORKING TODO
                #if g == 0 or b == 0:
                #    pbar.update()
                #    continue
                #max_height = np.log(float(g)/255) - np.log(1-float(g)/255)
                #min_height = np.log(float(b)/255) - np.log(1-float(b)/255)


                #variance_height -= mean_height

                x = ((float(i) - x_offset)/self.pixels_per_meter)
                y = ((float(j) - y_offset)/self.pixels_per_meter)

                if (r == 0):
                    pbar.update()
                    continue

                #if points_offset == 0:
                    #self.points.append([x,y,min_height])
                    #pbar.update()
                    #continue

                #if np.fabs(max_height - min_height) > (self.range[1] - self.range[0]):
                #    pbar.update()
                #    self.points.append([x,y,0)
                ##continue

                #if g == 0 and b == 0:
                #    print(mean_height, std_height, "A")
                #    pbar.update()
                #    continue
                mean_height = ((float(b)-127)/255) * (self.range[1]-self.range[0])
                std_height = ((float(g))/255) * (self.range[1]-self.range[0])
                #print (mean_height, std_height)

                #TODO calculate number of samples according something
                #TODO USE r value
                for _ in range(3):
                    z = random.uniform(mean_height - std_height, mean_height + std_height)
                    self.points.append([x,y,z])
                    self.rgb.append([r,g,b])
                #self.points.append([x,y,mean_height])
                #if b == g:
                #    print("NO height")
                #    pbar.update()
                #    continue

                #for w in np.arange(min_height, max_height, z_scaler):
                #    self.points.append([x,y,w])

                pbar.update()

                #
        print("number of points %d "% len(self.points))
        print("number of colors %d "% len(self.rgb))

        #self.viewer.close()
        #print np.unique(rgb[:,0])
        #print np.unique(rgb[:,1])
        #print np.unique(rgb[:,2])
        self.points = np.array(self.points).reshape(len(self.points),3)
        #rgb = pptk.rand(len(self.points), 3)
        if self.enable_ros:
            self.test()
        else:
            self.rgb = np.array(self.rgb, np.float64).reshape(len(self.rgb),3)
            print (np.mean(self.rgb))
            self.rgb /= 255
            self.viewer = pptk.viewer(self.points,self.rgb)
            self.viewer.set(point_size=0.05)

        if self.index > -1:
            if not self.enable_ros:
                raw_input("Press Enter to continue...")
        else:
            time.sleep(1)

        if self.task_done:
            if not self.enable_ros:
                self.viewer.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--folder", "-b", help="set input bagname", default=False)
    parser.add_argument("--extension", "-e", help="set image extension", default=".jpeg")
    parser.add_argument("--topic", "-t", default="/velodyne_points")
    parser.add_argument("--index", "-i", default=0)
    parser.add_argument("--ros", "-r", default=0)


    args = parser.parse_args()
    image2PC = ImageToPc(extension=args.extension, folder=args.folder, topic = args.topic, index = args.index, enable_ros = args.ros)

    while True:
        current_image = image2PC.get_next_image()
        if current_image is None:
            break

        image2PC.compute_pc(current_image)

        if image2PC.task_done:
            break

    print("Finished")

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
import matplotlib.pyplot as plt

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
        else:
            self.feature_x = list()
            self.feature_y = list()
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
        if self.index == 0:
            img_name = os.path.join(self.folder , str(self.counter)+self.extension) #self.transformed_image.header.stamp
            self.counter = self.counter + 25
        else:
            img_name = os.path.join(self.folder , str(self.index)+self.extension) #self.transformed_image.header.stamp
            self.task_done = True
            self.counter = self.counter + 1

        try:
            #im_raw = Image.open(img_name)
            #im = np.asarray(im_raw)
            im = cv2.imread(img_name, cv2.IMREAD_COLOR)
            #im = cv2.imread(img_name)
            #im = cv2.cvtColor(im, cv2.COLOR_RGB2HSV).astype(np.float64)


            if im is None:
                print "No image", img_name
                self.task_done = True
            return im
        except cv2.error as e:
            print ("ERROR")
            self.task_done = True
            return None

    def close(self):
        if not self.enable_ros:
            print "AAA"
            plt.figure()
            print self.feature_x.shape
            print self.mask.shape
            plt.scatter(self.feature_x, self.feature_y, c=self.mask)

            gb = np.asarray(self.p)
            #plt.scatter(gb[:,0], gb[:,0],s=0.2, marker="o")

            plt.show(False)
            raw_input("Press Enter to continue...")

        if self.task_done:
            if not self.enable_ros:
                self.viewer.close()

    def evaluate(self):
        m_l = -1.2
        b_l = 0.3
        self.feature_x = np.asarray(self.feature_x)
        self.feature_y = np.asarray(self.feature_y)
        eval = m_l*self.feature_x - self.feature_y + b_l
        mask = np.ma.masked_greater(eval, 0.0).mask
        #masked_col = np.ma.masked_greater(measured_x, threshold).mask
        #mask = masked_row * masked_col
        self.mask = np.array(mask, dtype=np.str)
        self.mask[self.mask == 'False'] = 'b'
        self.mask[self.mask == 'True'] = 'r'


    def evaluate_point(self,x,y):
        m_l = -1.2#1.5
        b_l = 0.3
        eval = m_l*x - y + b_l
        return eval <0

    def compute_pc(self, img):
        height, width, channels = img.shape
        x_offset = height/2
        y_offset = width/2
        print("computing %d points ", height * width)
        pbar = pyprind.ProgBar(height*width)
        self.points = list()
        self.rgb = list()
        self.mask = list()
        self.p = list()

        z_scaler =  np.fabs(self.range[1]-self.range[0])/255
        full_range = self.range[1]-self.range[0]
        meters_per_pixel = full_range/height

        #mean_img = np.mean(np.asarray(img), axis=0)
        #std_mean = np.mean(np.asarray(img), axis=1)

        #masked_row = np.ma.masked_greater(row_mean[:,2], threshold).mask
        #masked_col = np.ma.masked_greater(col_mean[:,2], threshold).mask
        #mask = masked_row * masked_col
        #self.mask = np.array(mask, dtype=np.str)
        #self.mask[self.mask == 'False'] = 'b'
        #self.mask[self.mask == 'True'] = 'r'



        #row_of_interest = np.where(row_mean[:,2] > threshold)[0]
        #col_of_interest = np.where(col_mean[:,2] > threshold)[0]

        #print row_of_interest
        #print col_of_interest
        #for row in row_of_interest:
        #    for col in col_of_interest:
        #         print np.asarray(self.rgb).shape#[row,col]# = [255,255,255]

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
                mean_height = ((float(g)-127)/255) * full_range
                std_height = ((float(b))/255) * full_range
                #print (mean_height, std_height)
                self.feature_x.append(mean_height)
                self.feature_y.append(std_height)

                #self.p.append([mean_height, std_height])
                class_flag = self.evaluate_point(mean_height, std_height)
                #TODO calculate number of samples according something
                #TODO USE r value
                for _ in range(r):
                    s_x = random.uniform(x -meters_per_pixel, x + meters_per_pixel)
                    s_y = random.uniform(y -meters_per_pixel, y + meters_per_pixel)
                    z = random.uniform(mean_height - std_height, mean_height + std_height)
                    self.points.append([s_x,s_y,z])
                    if class_flag:#i in row_of_interest or j in col_of_interest:
                        #if std_height > 1:
                        self.rgb.append([255,255,255])
                        #self.mask.append('r')
                    else:
                        #self.rgb.append([r,g,b])
                        self.rgb.append([0,0,0])
                    #self.mask.append('b')
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
        self.evaluate()

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

            #for i in row_of_intestest:
                #col_mean = np.mean(np.asarray(img), axis=1)
                #print (np.mean(col_mean[:,2] > 0.5))[1]

            self.rgb /= 255
            self.viewer = pptk.viewer(self.points,self.rgb)
            self.viewer.set(point_size=0.05)

        if not self.index > -1:
            time.sleep(1)



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

    image2PC.close()

    print("Finished")

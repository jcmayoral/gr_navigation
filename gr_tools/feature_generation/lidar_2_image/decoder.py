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
import random
import rospy
import ros_numpy
import pcl
import matplotlib.pyplot as plt
from sklearn.cluster import KMeans, AffinityPropagation, DBSCAN

help_text = 'This is a script that converts RGB images to PointCloud2 messages'


class ImageToPc():
    def __init__(self, extension, folder, topic=None, index = -1, enable_ros= 1, ground_truth = 0, debug_mode=0):
        #rospy.init_node("pointcloud_decoder")
        self.index = int(index)
        self.ground_truth = int(ground_truth)
        self.folder = folder
        self.extension = extension
        self.task_done = False
        self.viewer = None
        self.debug_mode = int(debug_mode)
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
        self.load_params()

        if self.ground_truth:
            try:
                os.mkdir("results_" + self.folder)
            except OSError:
                print ("Creation of the directory failed")
            else:
                print ("Successfully created the directory ")


    def save_results(self):
        #points_2d = self.points[:10,:-1]
        #print points_2d.shape
        #km = k_means(points_2d, n_clusters=10)
        #km = KMeans() #DBSCAN()#AffinityPropagation()
        #km.fit(np.asarray(self.points_2_cluster))
        pcl_pc = pcl.PointCloud(np.asarray(self.points_2_cluster, dtype = np.float32))
        tree = pcl_pc.make_kdtree()
        ec = pcl_pc.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.3)
        ec.set_MinClusterSize(0)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        cluster_centers = list()


        for j, indices in enumerate(cluster_indices):
            # cloudsize = indices
            #print('indices = ' + str(len(indices)))
            # cloudsize = len(indices)
            center = np.zeros(3)
            # points = np.zeros((cloudsize, 3), dtype=np.float32)
            # for indice in range(len(indices)):
            for i, indice in enumerate(indices):
                # print('dataNum = ' + str(i) + ', data point[x y z]: ' + str(cloud_filtered[indice][0]) + ' ' + str(cloud_filtered[indice][1]) + ' ' + str(cloud_filtered[indice][2]))
                # print('PointCloud representing the Cluster: ' + str(cloud_cluster.size) + " data points.")
                #points[i][0] = cloud_filtered[indice][0points[i][1] = cloud_filtered[indice][1]
                #points[i][2] = cloud_filtered[indice][2]
                #print i, indice
                center += self.points_2_cluster[indice]
            print j, center/len(indices)
            cluster_centers.append(copy.copy(center/len(indices)))

        f = open(os.path.join("results_" + self.folder, str(self.folder) + str(self.index)),"w")
        f.seek(0)
        for i in cluster_centers:
            f.write( str(i)+"\n" )
        f.close()

    def test(self):
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

        msg = ros_numpy.msgify(PointCloud2, data)
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "velodyne"
        self.publisher.publish(msg)

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

        if self.debug_mode:
            print ("Running debug mode")
            self.task_done = True
            #self.index = 1

        img_name = os.path.join(self.folder , str(self.index)+self.extension)
        self.index = self.index + 1

        try:
            im = cv2.imread(img_name, cv2.IMREAD_COLOR)

            if im is None:
                print "No image", img_name
                self.task_done = True
                self.close()
            return im
        except cv2.error as e:
            print ("ERROR")
            self.task_done = True
            return None

    def close(self):
        if not self.enable_ros:
            plt.clf()
            plt.cla()
            plt.scatter(self.feature_x, self.feature_y, c=self.mask)
            plt.show(False)
            time.sleep(2.0)
            plt.close()
            if self.viewer is not None:
                self.viewer.clear()
                self.feature_x = list()
                self.feature_y = list()
            #raw_input("Press Enter to continue...")

        if self.task_done:
            if not self.enable_ros:
                self.viewer.close()

    def evaluate(self):
        m_l = -1
        b_l = -1.4
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
        m_l = -1#1.5
        b_l = -1.4
        eval = m_l*x - y + b_l
        return eval <0

    def compute_pc(self, img):
        height, width, channels = img.shape
        x_offset = height/2
        y_offset = width/2
        print("computing %d points ", height * width)
        pbar = pyprind.ProgBar(height*width)
        self.points = list()
        self.points_2_cluster = list()
        self.rgb = list()
        self.mask = list()
        self.feature_x = list()
        self.feature_y = list()


        z_scaler =  np.fabs(self.range[1]-self.range[0])/255
        full_range = self.range[1]-self.range[0]
        meters_per_pixel = full_range/height

        for i in range(height):
            for j in range(width):
                #r counter
                #g mean
                #b variance
                r,g,b = copy.copy(img[i,j,:])

                x = ((float(i) - x_offset)/self.pixels_per_meter)
                y = ((float(j) - y_offset)/self.pixels_per_meter)

                if (r == 0):
                    pbar.update()
                    continue

                mean_height = ((float(g)-127)/255) * full_range
                std_height = ((float(b))/255) * full_range

                self.feature_x.append(mean_height)
                self.feature_y.append(std_height)
                class_flag = self.evaluate_point(mean_height, std_height)

                for _ in range(r):
                    s_x = random.uniform(x -meters_per_pixel, x + meters_per_pixel)
                    s_y = random.uniform(y -meters_per_pixel, y + meters_per_pixel)
                    z = random.uniform(mean_height - std_height, mean_height + std_height)
                    self.points.append([s_x,s_y,z])
                    if class_flag:
                        self.rgb.append([255,255,255])
                    else:
                        self.rgb.append([r,g,b])

                #Last point of col goest to cluster with mean_height
                if class_flag:
                    self.points_2_cluster.append([s_x, s_y, mean_height])

                pbar.update()

        print("number of points %d "% len(self.points))
        print("number of colors %d "% len(self.rgb))

        self.evaluate()

        self.points = np.array(self.points).reshape(len(self.points),3)


        if self.ground_truth:
            self.save_results()

        else:
            if self.viewer is not None:
                self.close()
            if self.enable_ros:
                self.test()
            else:
                self.rgb = np.array(self.rgb, np.float64).reshape(len(self.rgb),3)

                self.rgb /= 255

                if self.viewer is None:
                    self.viewer = pptk.viewer(self.points,self.rgb)
                    self.viewer.set(point_size=0.05)
                else:
                    self.viewer.load(self.points,self.rgb)


            if self.index != 0:
                self.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description = help_text)
    parser.add_argument("--folder", "-b", help="set input bagname", default=False)
    parser.add_argument("--extension", "-e", help="set image extension", default=".jpeg")
    parser.add_argument("--topic", "-t", default="/velodyne_points")
    parser.add_argument("--index", "-i", default=0)
    parser.add_argument("--ros", "-r", default=0)
    parser.add_argument("--ground_truth", "-gt", default=0)
    parser.add_argument("--debug", "-db", default=0)

    args = parser.parse_args()
    image2PC = ImageToPc(extension=args.extension, folder=args.folder, topic = args.topic,
                         index = args.index, enable_ros = args.ros,
                         ground_truth = args.ground_truth, debug_mode = args.debug)

    while True:
        current_image = image2PC.get_next_image()
        if current_image is None:
            break

        image2PC.compute_pc(current_image)

        if image2PC.task_done:
            break

    print("Finished")

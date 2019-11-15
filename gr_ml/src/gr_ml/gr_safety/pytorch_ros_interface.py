#!/usr/bin/python
import rospy
from nn_model import NetworkModel
from safety_msgs.msg import FoundObjectsArray, SafetyState
from jsk_recognition_msgs.msg import BoundingBoxArray

from std_msgs.msg import Bool
import numpy as np
import torch
import collections

class PyTorchWrapper():

    def __init__(self, batchsize=50, epochs=2, max_training_cycles = 10):
        self.network = NetworkModel() #.to(torch.device("cpu"))
        self.is_training = True
        self.current_batch = 0
        self.batch_size = batchsize
        self.epochs = epochs
        self.danger_dist = 2.0
        self.warn_dist = 3.5
        self.current_training = 0
        self.max_training_cycles = max_training_cycles
        #Current version just using poses
        features_number = 7
        self.processed_array = np.zeros((batchsize,features_number))
        self.labels = np.zeros((batchsize,3))
        #id 2
        id = 2
        self.pc_fit_subscriber = rospy.Subscriber("/detection/bounding_boxes", BoundingBoxArray,self.fit_cb2, queue_size=1,callback_args=(id,1))
        #id 1
        id = 1
        self.fit_subscriber = rospy.Subscriber("/found_object", FoundObjectsArray, self.fit_cb, queue_size=1,callback_args=(id,1))
        self.mode_subscriber = rospy.Subscriber("mode_selector" , Bool, self.mode_cb)
        self.safety_monito_pub = rospy.Publisher("/observer_0" , SafetyState)

        print ("pytorch wrapper constructor")

    def fit_cb2(self, msg, args):
        #print ("pointcloud")
        id = args[0]
        for j in msg.boxes:
            #print j.class_name
            p = j.pose.position
            #This is just simple version
            self.processed_array[self.current_batch] = [p.x,p.y,p.z,j.dimensions.x, j.dimensions.y, j.dimensions.z,id]
            dist = np.sqrt(np.power(p.x,2) + np.power(p.y,2) + np.power(p.z,2) )
            self.labels[self.current_batch] = [int(dist>self.warn_dist),  int(self.danger_dist<dist<self.warn_dist), int(dist<self.danger_dist)]
            self.current_batch = self.current_batch + 1

            #TODO Add skipped valyes to queue
            if self.current_batch == self.batch_size:
                self.current_batch = 0
                if self.is_training:
                    #TODO check if possible to partial_fit
                    #print (self.labels)
                    labels_count = np.count_nonzero(self.labels, axis=0)
                    print (labels_count, np.any(labels_count == 0))
                    if np.any(labels_count >= 0.7*self.batch_size):
                        print "Ignoring batch... high umbalanced"
                        return
                    #FOR NOW OWN all labels are random
                    #label = np.random.choice([0,1], size=(self.batch_size,1))
                    self.fit(self.processed_array, self.labels)
                    if self.current_training > self.max_training_cycles:
                        self.is_training = False
                    self.current_training = self.current_training + 1


            if not self.is_training:
                self.predict(np.asarray([p.x,p.y,p.z,j.dimensions.x, j.dimensions.y, j.dimensions.z,id]))
                return

    def fit_cb(self, msg, args):
        #print ("camera")

        id = args[0]
        #the fir n vaues of array
        #self.processed_array =  np.roll(self.proccesed_array,-len(msg.object),axis=1)

        for j in msg.objects:
            #print j.class_name
            flag = 0
            if j.class_name == 'person':
                flag = 1
            p = j.centroid.point
            #This is just simple version
            # TODO
            self.processed_array[self.current_batch] = [p.x,p.y,p.z,0,0,0,id]
            dist = np.sqrt(np.power(p.x,2) + np.power(p.y,2) + np.power(p.z,2) )
            self.labels[self.current_batch] = [int(dist>self.warn_dist),  int(self.danger_dist<dist<self.warn_dist), int(dist<self.danger_dist)]
            self.current_batch = self.current_batch + 1


            #TODO Add skipped valyes to queue
            if self.current_batch == self.batch_size:
                self.current_batch = 0
                if self.is_training:
                    #TODO check if possible to partial_fit
                    #print (self.labels)
                    #FOR NOW OWN all labels are random
                    #label = np.random.choice([0,1], size=(self.batch_size,1))
                    self.fit(self.processed_array, self.labels)
                    if self.current_training > self.max_training_cycles:
                        self.is_training = False
                    self.current_training = self.current_training + 1


            if not self.is_training:
                self.predict(np.asarray([p.x,p.y,p.z,0,0,0, id]))
                return


    def mode_cb(self,msg):
        self.is_training = msg.data

    def fit(self,data, target):
        for epoch in range(self.epochs):
            output = self.network(torch.from_numpy(data).float())
            self.network.loss = self.network.criterion(output, torch.from_numpy(target).float())
            self.network.loss.mean().backward()
            self.network.optimizer.step()
            if (epoch % 10 == 0):
                print("Epoch {} - loss: {}".format(epoch, self.network.loss.mean()))

    def predict(self,data):
        output = self.network(torch.from_numpy(data).float())
        print("prediction",output.detach().numpy())
        fb = SafetyState()
        fb.msg.data = "ML"
        fb.mode = np.argmax(output.detach().numpy())
        self.safety_monito_pub.publish(fb)

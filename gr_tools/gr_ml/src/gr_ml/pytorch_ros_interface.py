#!/usr/bin/python
import rospy
from nn_model import NetworkModel
from safety_msgs.msg import FoundObjectsArray
from std_msgs.msg import Bool
import numpy as np
import torch
import collections

class PyTorchWrapper():

    def __init__(self, batchsize=10, epochs=100):
        self.network = NetworkModel() #.to(torch.device("cpu"))
        self.is_training = True
        self.current_batch = 0
        self.batch_size = batchsize
        self.epochs = epochs
        #Current version just using poses
        self.processed_array = np.zeros((batchsize,3))
        self.fit_subscriber = rospy.Subscriber("/found_object", FoundObjectsArray, self.fit_cb, queue_size=1)
        self.mode_subscriber = rospy.Subscriber("mode_selector" , Bool, self.mode_cb)
        print ("keras wrapper constructor")
    
    def fit_cb(self, msg):
        #the fir n vaues of array 
        #self.processed_array =  np.roll(self.proccesed_array,-len(msg.object),axis=1)

        #TODO check if possible to partial_fit on Keras
        if not self.is_training:
            return
            
        for j in msg.objects:
            #print j.class_name
            p = j.centroid.point
            #This is just simple version
            self.processed_array[self.current_batch] = [p.x,p.y,p.z]
            self.current_batch = self.current_batch + 1
            #TODO Add skipped valyes to queue
            if self.current_batch == self.batch_size-1:
                self.current_batch = 0
                print self.processed_array
                #FOR NOW OWN all labels are random
                label = np.random.choice([0,1], size=self.batch_size)
                self.fit(self.processed_array, label)

    def mode_cb(self,msg):
        self.is_training = msg.data

    def fit(self,data, target):
        for epoch in range(self.epochs):
            output = self.network(torch.from_numpy(data).float())
            self.network.loss = self.network.criterion(output, torch.from_numpy(target).float())
            self.network.loss.mean().backward()
            self.network.optimizer.step()
            if (epoch % 10 == 0):
                print("Epoch {} - loss: {}".format(epoch, self.network.loss.sum()))
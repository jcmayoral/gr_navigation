#!/usr/bin/python
import rospy
from nn_model import NetworkModel
from safety_msgs.msg import FoundObjectsArray, SafetyState
from std_msgs.msg import Bool
import numpy as np
import torch
import collections

class PyTorchWrapper():

    def __init__(self, batchsize=25, epochs=10, max_training_cycles = 5):
        self.network = NetworkModel() #.to(torch.device("cpu"))
        self.is_training = True
        self.current_batch = 0
        self.batch_size = batchsize
        self.epochs = epochs
        self.current_training = 0
        self.max_training_cycles = max_training_cycles
        #Current version just using poses
        self.processed_array = np.zeros((batchsize,3))
        self.labels = np.zeros((batchsize,2))
        self.fit_subscriber = rospy.Subscriber("/found_object", FoundObjectsArray, self.fit_cb, queue_size=1)
        self.mode_subscriber = rospy.Subscriber("mode_selector" , Bool, self.mode_cb)
        self.safety_monito_pub = rospy.Publisher("/observer_0" , SafetyState)

        print ("keras wrapper constructor")
    
    def fit_cb(self, msg):
        #the fir n vaues of array 
        #self.processed_array =  np.roll(self.proccesed_array,-len(msg.object),axis=1)
            
        for j in msg.objects:
            #print j.class_name
            p = j.centroid.point
            #This is just simple version
            self.processed_array[self.current_batch] = [p.x,p.y,p.z]
            dist = np.sqrt(np.power(p.x,2) + np.power(p.y,2) + np.power(p.z,2) )
            self.labels[self.current_batch] = [int(dist>1.0), int(dist<1.0)]
            self.current_batch = self.current_batch + 1

            #TODO Add skipped valyes to queue
            if self.current_batch == self.batch_size:
                self.current_batch = 0
                if self.is_training:
                    #TODO check if possible to partial_fit 
                    print (self.labels)
                    #FOR NOW OWN all labels are random
                    #label = np.random.choice([0,1], size=(self.batch_size,1))
                    self.fit(self.processed_array, self.labels)
                    if self.current_training > self.max_training_cycles:
                        self.is_training = False
                    self.current_training = self.current_training + 1


            if not self.is_training:
                    self.predict(np.asarray([p.x,p.y,p.z]))
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
        fb.mode = int(output.detach().numpy()[1]>0.5)
        self.safety_monito_pub.publish(fb)


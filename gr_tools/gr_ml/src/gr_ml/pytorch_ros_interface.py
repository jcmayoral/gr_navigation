#!/usr/bin/python
import rospy
from nn_model import NetworkModel
from safety_msgs.msg import FoundObjectsArray
from std_msgs.msg import Bool
import numpy as np
import torch
import collections

class PyTorchWrapper():

    def __init__(self, batchsize):
        self.network = NetworkModel() #.to(torch.device("cpu"))
        self.is_training = True
        self.current_batch = 0
        self.batch_size = batchsize
        #Current version just using poses
        self.processed_array = np.zeros((batchsize,3))
        self.fit_subscriber = rospy.Subscriber("/found_object", FoundObjectsArray, self.fit_cb)
        self.mode_subscriber = rospy.Subscriber("mode_selector" , Bool, self.mode_cb)
        print ("keras wrapper constructor")
    
    def fit_cb(self, msg):
        #the fir n vaues of array 
        #self.processed_array =  np.roll(self.proccesed_array,-len(msg.object),axis=1)

        #TODO check if possible to partial_fit on Keras
        if not self.is_training:
            #TODO REVIEW
            break

        for j in msg.objects:
            #print j.class_name
            p = j.centroid.point
            #This is just simple version
            self.processed_array[self.current_batch] = [p.x,p.y,p.z]
            self.current_batch = self.current_batch + 1
            #TODO Add skipped valyes to queue
            if self.current_batch == self.batch_size:
                print processed_array
                #FOR NOW OWN all labels are 1
                label = np.ones(len(batch_size))
                self.fit(processed_array, label)
                self.current_batch = 0

predict(processed_array))

    def mode_cb(self,msg):
        self.is_training = msg.data

    def fit(self,data, target, epoch = 1):
        #self.train()
        #data, target = data.to(self.device), target.to(self.device)
        #self.network.optimizer.zero_grad()
        output = self.network(torch.from_numpy(data).float())
        print output
        #loss = F.nll_loss(output, target)
        #loss.backward()
        #self.network.optimizer.step()
        #if batch_idx % 1 == 0:
        #    print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
        #        epoch, batch_idx * len(data), len(train_loader.dataset),
        #        100. * batch_idx / len(train_loader), loss.item()))
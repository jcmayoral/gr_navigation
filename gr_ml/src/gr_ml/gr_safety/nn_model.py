#!/usr/bin/python
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim


#Based on tutorial example https://github.com/pytorch/examples/blob/master/mnist/main.py
# and https://medium.com/@tomgrek/building-your-first-neural-net-from-scratch-with-pytorch-56b0e9c84d54
class NetworkModel(nn.Module):
    def __init__(self):
        super(NetworkModel, self).__init__()
        self.device = torch.device("cpu")
        #fully conected layer 3 inputs 3 output
        self.fc1 = nn.Linear(7,16)
        self.r1 = nn.ReLU()
        self.fc2 = nn.Linear(16,8)
        self.r2 = nn.ReLU()
        self.fc3 = nn.Linear(8,3)
        self.sm = nn.Softmax()

        self.optimizer = optim.SGD(self.parameters(), lr=0.01, momentum=0.2)
        self.optimizer.zero_grad()

    def forward(self, x):
        x = self.sm(self.fc3(self.r2(self.fc2(self.r1(self.fc1(x))))))
        return x

    def criterion(self, out, label):
        criteria = nn.NLLLoss()

        return (label - out)**2

class TerrainNetworkModel(nn.Module):
    def __init__(self):
        super(TerrainNetworkModel, self).__init__()
        self.device = torch.device("cpu")
        self.cc1 = nn.Conv2d(20, 10, kernel_size=(3,3), stride=2, padding=1, padding_mode='same')
        self.r1 = nn.ReLU()
        self.pool1 = torch.nn.MaxPool2d((1,1))#kernel_size=9, stride=1, padding=2)
        self.fc1 = torch.nn.Linear(200, 3)
        #self.r2 = nn.ReLU()
        #64 input features, 10 output featres for our 3 defined classes
        #self.cc2 = torch.nn.MaxPool2d(3)#nn.Conv2d(1, 1, kernel_size=(5,5), stride=1, padding=3, padding_mode='same')
        #self.fc2 = torch.nn.Linear(10, 3)
        self.sm = nn.Softmax()

        self.optimizer = optim.SGD(self.parameters(), lr=0.01, momentum=0.2)
        self.optimizer.zero_grad()

    def forward(self, x):
        print(x.shape)
        o1 = self.r1(self.cc1(x))
        print (o1.shape)
        o2 = self.pool1(o1)
        print (o2.shape)
        o2 = o2.flatten()
        o3 = self.fc1(o2)
        #print (o3.shape)
        o4 = self.sm(o3)
        print (o4.shape)
        #o5 = self.sm(o4)
        #print (o5.shape)
        return o4

    def criterion(self, out, label):
        criteria = nn.NLLLoss()
        return (label - out)**2

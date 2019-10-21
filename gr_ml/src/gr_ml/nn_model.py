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
        self.fc1 = nn.Linear(3,8)
        self.r1 = nn.ReLU()
        self.fc2 = nn.Linear(8,3)
        self.sm = nn.Softmax()

        self.optimizer = optim.SGD(self.parameters(), lr=0.01, momentum=0.5)
        self.optimizer.zero_grad()

    def forward(self, x):
        x = self.sm(self.fc2(self.r1(self.fc1(x))))
        return x

    def predict(self,msg):
        print "predict "
    
    def criterion(self, out, label):
        criteria = nn.NLLLoss()

        return (label - out)**2
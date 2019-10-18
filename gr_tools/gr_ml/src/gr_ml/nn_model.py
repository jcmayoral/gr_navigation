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
        #self.optimizer = optim.SGD(self.parameters(), lr=0.01, momentum=0.5)
        self.fc1 = nn.Linear(3,1)

    def forward(self, x):
        x = self.fc1(x)
        return x

    def predict(self,msg):
        print "predict "


"""
    def fit(self,data, target, epoch = 1):
        self.train()
        #data, target = data.to(self.device), target.to(self.device)
        self.optimizer.zero_grad()
        output = self(data)
        loss = F.nll_loss(output, target)
        loss.backward()
        self.optimizer.step()
        if batch_idx % 1 == 0:
            print('Train Epoch: {} [{}/{} ({:.0f}%)]\tLoss: {:.6f}'.format(
                    epoch, batch_idx * len(data), len(train_loader.dataset),
                    100. * batch_idx / len(train_loader), loss.item()))


    def test(args, model, device, test_loader):
        model.eval()
        test_loss = 0
        correct = 0
        with torch.no_grad():
            for data, target in test_loader:
                data, target = data.to(device), target.to(device)
                output = model(data)
                test_loss += F.nll_loss(output, target, reduction='sum').item() # sum up batch loss
                pred = output.argmax(dim=1, keepdim=True) # get the index of the max log-probability
                correct += pred.eq(target.view_as(pred)).sum().item()

        test_loss /= len(test_loader.dataset)

        print('\nTest set: Average loss: {:.4f}, Accuracy: {}/{} ({:.0f}%)\n'.format(
            test_loss, correct, len(test_loader.dataset),
            100. * correct / len(test_loader.dataset)))
"""
#!/usr/bin/python
import keras


class NetworkModel:
    def __init__(self):
        print "NN"
    
    def fit(self, x, label):
        print "FIT ", x , label

    def predict(self,msg):
        print "predict "
#!/usr/bin/python
import rospy
from nn_model import NetworkModel
from safety_msgs.msg import FoundObjectsArray
from std_msgs.msg import Bool

class KerasWrapper(NetworkModel):

    def __init__(self):
        self.is_training = True
        self.fit_subscriber = rospy.Subscriber("/found_object", FoundObjectsArray, self.fit_cb)
        self.mode_subscriber = rospy.Subscriber("mode_selector" , Bool, self.mode_cb)
        print ("keras wrapper constructor")
    
    def fit_cb(self, msg):
        processed_array = msg
        #FOR NOW OWN all labels are 1
        label = 1
        #TODO check if possible to partial_fit on Keras
        if self.is_training:
            self.fit(processed_array, label)
        else:
            print (self.predict(processed_array))

    def mode_cb(self,msg):
        self.is_training = msg.data
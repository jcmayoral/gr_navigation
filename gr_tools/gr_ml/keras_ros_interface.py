import rospy
from nn_model import NetworkModel
from safety_msgs.msg import FoundObjectArray
from std_msgs.msg import Bool

class KerasWrapper(NetworkModel):

    def __init__(self):
        self.is_training = True
        self.fit_subscriber = rospy.Subscriber("/found_object", FoundObjectArray, self.fit_cb)
        self.mode_subscriber = rospy.Subscriber("mode_selector" , Bool, self.mode_cb)
    
    def fit_cb(self, msg):
        processed_array = msg
        #FOR NOW OWN all labels are 1
        label = 1
        #TODO check if possible to partial_fit on Keras
        if is_training:
            self.fit(processed_array, label)
        else:
            print (self.predict(processed_array))

    def mode_cb(self,msg):
        self.is_training = msg.data
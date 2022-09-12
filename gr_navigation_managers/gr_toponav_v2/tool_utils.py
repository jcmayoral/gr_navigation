import rospy
from std_msgs.msg import Int16, String

class ToolInterface:
    def __init__(self, topic=None, type_str=None, high_value=0, low_value=0):
        print ("TOPIC {} type{}".format(topic, type_str))
        self.high_value = high_value
        self.low_value = low_value
        self.type = self.create_type(type_str)
        if topic is not None:
            self.pub = rospy.Publisher(topic, self.type, queue_size = 1)
        else:
            self.pub = None

    def start(self):
        if self.pub is None:
            return
        self.pub.publish(self.type(data=self.high_value))

    def stop(self):
        if self.pub is None:
            return
        self.pub.publish(self.type(data=self.low_value))

    def create_type(self,type):
        if type == "int":
            return Int16
        if type == "string":
            return String
        if type is None:
            return None
        else:
            raise ValueError("type " + type + "not recognized")

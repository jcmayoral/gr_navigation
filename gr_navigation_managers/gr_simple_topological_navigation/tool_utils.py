import rospy
from std_msgs.msg import Int16, String|

class ToolInterface:
    def __init__(self, topic, type_str, high_value, low_value):
        print ("TOPIC {} type{}".format(topic, type_str))
        self.high_value = high_value
        self.low_value = low_value
        self.type = self.create_type(type_str)
        self.pub = rospy.Publisher(topic, self.type, queue_size = 1)

    def start(self):
        self.pub.publish(self.type(data=self.high_value))

    def stop(self):
        self.pub.publish(self.type(data=self.low_value))

    def create_type(self,type):
        if type == "int":
            return Int16
        if type == "string":
            return String
        else:
            raise ValueError("type " + type + "not recognized")
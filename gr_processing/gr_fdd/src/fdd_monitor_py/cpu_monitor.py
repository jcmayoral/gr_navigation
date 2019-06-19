import rospy
import psutil
from std_msgs.msg import Float32

class CPUMonitor:
    def __init__(self):
        rospy.init_node("cpu_monitor")
        self.cpu_pub = rospy.Publisher("/cpu_consumption", Float32)
        self.rate = rospy.Rate(10) # 10hz

    def publish_cpu(self):
        self.cpu_pub.publish(Float32(data=psutil.cpu_percent()))
        self.rate.sleep()

import rospy
from geometry_msgs.msg import Twist

class SimpleCropNavController:
    def __init__(self, desired_speed = 1.0):
        self.pub = rospy.Publisher("/nav_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.twist.linear.x = desired_speed
        rospy.Timer(rospy.Duration(10), self.change_direction)

    def publish(self):
        self.pub.publish(self.twist)

    def change_direction(self,event):
        print ("chnage direction")
        self.twist.linear.x = -self.twist.linear.x

import rospy
import numpy as np
from message_filters import Subscriber, ApproximateTimeSynchronizer
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
from copy import deepcopy

class WheelSyncronizer:
    def __init__(self):
        rospy.init_node("wheels_sync")
        #values copied from the diffdrive_controller file not available on parameter server
        self.R = 0.5
        self.L = 0.512#rospy.get_param("/filtered/canopen_interface/differential_drive/wheel_separation")
        self.last_odom = Odometry()
        self.error_vel = Twist()
        self.computed_twist =  Twist()

        self.odom_vel_pub = rospy.Publisher("recomputed_odom_vel", Twist, queue_size = 1)
        self.diff_pub = rospy.Publisher("magazino_fdd/twist_error", Twist, queue_size = 1)
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb, queue_size = 1)
        self.tss = ApproximateTimeSynchronizer([Subscriber("/left_speed",Float64), Subscriber("/right_speed", Float64)],5,0.1, allow_headerless=True)
        self.tss.registerCallback(self.got_velocities)

    def odom_cb(self, msg):
        self.last_odom = msg
        self.error_vel.linear.x = self.computed_twist.linear.x - self.last_odom.twist.twist.linear.x
        self.error_vel.angular.z = self.computed_twist.angular.z - self.last_odom.twist.twist.angular.z
        self.diff_pub.publish(self.error_vel)

    def got_velocities(self, left_wheel, right_wheel):
        self.computed_twist.linear.x = self.R *(left_wheel.data + right_wheel.data)/2
        self.computed_twist.angular.z = self.R/self.L * (right_wheel.data - left_wheel.data)
        self.odom_vel_pub.publish(self.computed_twist)




"""
input v,w

#CYCLE
-x- = v cos phy
-y- = v sin phy
-phy- = w

#DIFF DRIVE
-x- = R/2 (vr+vl) cos(phy)
-y- = R/2 (vr+vl) sin(phy)
w = R/L (vr-vl)

#Substitution
v = R/2 (vr+vl)
w = R/L (vr-vl)

vr = 2v/R - vl
vl = vr - Lw/R

vr = (2v + Lw)/2R
vl = (2v - Lw)/2R

#Inverting
2R vr - Lw = 2v
2v - 2R vl =  Lw

2R (vr + vl) = 4v
2R (vr - vl) = 2Lw

"""

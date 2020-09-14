import rospy
import gr_topological_navigation.states.utils as utils
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import tf
import sys
import math

class SimpleCropNavController:
    def __init__(self, desired_speed = 1.0):
        self.twist = Twist()
        self.twist.linear.x = desired_speed
        self.listener = tf.TransformListener()
        self.max_motions = 20
        self.current_motions = 0

        self.startpose = [0,0]
        self.endpose = [0,0]
        self.currentpose = [0,0]
        self.start = False
        self.forward = True

        try:
            self.listener.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(2.0))
            (trans,rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time())
            #print type(trans),rot
            self.startpose = [trans[0], trans[1]]
            #print self.startpose
            endpose = PoseStamped()
            endpose.header.frame_id = "base_link"
            endpose.pose.orientation.w = 1.0
            endpose.pose.position.x = 10.0
            p_end = self.listener.transformPose("odom", endpose)
            self.endpose = [p_end.pose.position.x, p_end.pose.position.y]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            sys.exit()#continue
        #rospy.Timer(rospy.Duration(10), self.change_direction)
        rospy.Subscriber("/odometry/base_raw", Odometry, self.odom_cb)
        self.pub = rospy.Publisher("/nav_vel", Twist, queue_size=1)
        self.rb = utils.BagRecorder(desired_path = "/home/jose/ros_ws/src/gr_navigation/gr_navigation_managers/simple_crop_nav/data/",smach=False)

        if self.forward:
            self.rb.startBag()

    def odom_cb(self, msg):
        self.currentpose[0] = msg.pose.pose.position.x
        self.currentpose[1] = msg.pose.pose.position.y
        self.start = True

    def is_finished(self):
        return self.current_motions < self.max_motions

    def publish(self):
        self.pub.publish(self.twist)

        if self.forward:
            if math.sqrt(math.pow(self.endpose[0] - self.currentpose[0],2) + math.pow(self.endpose[1] - self.currentpose[1],2)) < 0.5:
                self.change_direction()
        else:
            if math.sqrt(math.pow(self.startpose[0] - self.currentpose[0],2) + math.pow(self.startpose[1] - self.currentpose[1],2)) < 0.5:
                self.change_direction()

    def change_direction(self):
        self.twist.linear.x = -self.twist.linear.x
        self.forward = not self.forward
        print ("chnage direction forward ", self.forward)
        self.current_motions = self.current_motions + 1

        if not self.forward:
            self.rb.close()
        else:
            self.rb.restart("test", str(self.current_motions), close_required = False)

import rospy
import gr_topological_navigation.states.utils as utils
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Bool
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from command_parser import parse_command
from jsk_gui_msgs.msg import VoiceMessage
import tf
import sys
import math

list_topics = {"/velodyne_points": PointCloud2, "/tf" : TFMessage, "/tf_statc" : TFMessage,
                "nav_vel" : Twist, "/safe_score": Float32,
                "/safe_nav_vel": Twist, "/lock_all" : Bool , "/grid_map": GridMap}

class SimpleCropNavController:
    def __init__(self, desired_speed = 1.0):
        self.twist = Twist()
        self.twist.linear.x = desired_speed
        self.listener = tf.TransformListener()
        self.initialize_test()

        rospy.Subscriber("/odometry/base_raw", Odometry, self.odom_cb)
        rospy.Subscriber("/Tablet/voice", VoiceMessage, self.voice_cb)
        self.pub = rospy.Publisher("/nav_vel", Twist, queue_size=1)
        self.rb = utils.BagRecorder(record_topics = list_topics,
                                    desired_path = "/home/jose/ros_ws/src/gr_navigation/gr_navigation_managers/simple_crop_nav/data/",
                                    smach=False, start=False)


        rospy.Timer(rospy.Duration(0.1), self.publish)
        #rospy.spin()

    def initialize_test(self):
        self.max_motions = 20
        self.current_motions = 0

        self.startpose = [0,0]
        self.endpose = [0,0]
        self.currentpose = [0,0]
        self.start = False
        self.forward = True
        self.command = None

    def voice_cb(self,msg):
        command = parse_command(msg.texts)
        print "command to execute ", command
        if command == "START_TEST":
            self.initialize_test()
            self.setPoses()
            self.rb.startBag()
        if command == "STOP_TEST":
            self.emergency_stop()

        self.command = command
        #self.setPose

    def setPoses(self):
        try:
            self.listener.waitForTransform('/odom', '/base_link', rospy.Time(), rospy.Duration(2.0))
            (trans,rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time())
            #print type(trans),rot
            self.startpose = [trans[0], trans[1]]
            #print self.startpose
            endpose = PoseStamped()
            endpose.header.frame_id = "base_link"
            endpose.pose.orientation.w = 1.0
            if self.forward:
                endpose.pose.position.x = 10.0
            else:
                endpose.pose.position.x = -10.0
            p_end = self.listener.transformPose("odom", endpose)
            self.endpose = [p_end.pose.position.x, p_end.pose.position.y]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            sys.exit()#continue


    def odom_cb(self, msg):
        self.currentpose[0] = msg.pose.pose.position.x
        self.currentpose[1] = msg.pose.pose.position.y
        self.start = True

    def is_running(self):
        return self.current_motions < self.max_motions

    def publish(self, event):
        if self.command is None:
            print "waiting for voice command"
            return

        self.pub.publish(self.twist)
        #print math.sqrt(math.pow(self.endpose[0] - self.currentpose[0],2) + math.pow(self.endpose[1] - self.currentpose[1],2))

        if math.sqrt(math.pow(self.endpose[0] - self.currentpose[0],2) + math.pow(self.endpose[1] - self.currentpose[1],2)) < 0.5:
            self.change_direction()

    def change_direction(self):

        self.twist.linear.x = -self.twist.linear.x
        self.forward = not self.forward
        print ("chnage direction forward ", self.forward)
        self.current_motions = self.current_motions + 1

        self.setPoses()

        if self.forward:
            self.rb.startBag()

        if not self.forward:
            self.rb.close()
        else:
            if self.is_running():
                rospy.loginfo("TEST "+ str(self.current_motions))
                self.rb.restart("test", str(self.current_motions), close_required = False)
            else:
                self.rb.close()

    def emergency_stop(self):
        self.rb.close()
        self.initializeTests()

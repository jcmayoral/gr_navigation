import rospy
import gr_topological_navigation.states.utils as utils
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32, Bool
from std_msgs.msg import String
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from grid_map_msgs.msg import GridMap
from command_parser import parse_command
from jsk_gui_msgs.msg import VoiceMessage
from safety_msgs.msg import FoundObjectsArray
import tf
import sys
import os
import math

import actionlib
from gr_action_msgs.msg import SingleRowExecutionAction, SingleRowExecutionFeedback, SingleRowExecutionResult
from gr_action_msgs.srv import GetMetrics, GetMetricsResponse, GetMetricsRequest

list_topics = {"/velodyne_points": PointCloud2, "/tf" : TFMessage, "/tf_statc" : TFMessage,
                "/nav_vel" : Twist, "/safe_score": Float32,
                "/safe_nav_vel": Twist, "/lock_all" : Bool , "/grid_map": GridMap,
                "/pointcloud_lidar_processing/found_object" : FoundObjectsArray
}

class SimpleRowNavController:
    def __init__(self, desired_speed = 0.5, folder = "data"):
        self.twist = Twist()
        self.twist.linear.x = desired_speed
        self.desired_speed = desired_speed
        self.is_next_required = False
        self.distance = 10.0
        self.repetitions = 20
        self.listener = tf.TransformListener()
        self.initialize_test()
        self.rb = utils.BagRecorder(record_topics = list_topics,
                                    desired_path = "/home/jose/ros_ws/src/gr_navigation/gr_navigation_actions/simple_row_nav/"+ folder +"/",
                                    enable_smach=False, start=False)

        rospy.Subscriber("/odometry/base_raw", Odometry, self.odom_cb)
        rospy.Subscriber("/Tablet/voice", VoiceMessage, self.voice_cb)
        rospy.Subscriber("fake_voice_command", String, self.voice_cb2)
        self.pub = rospy.Publisher("/nav_vel", Twist, queue_size=1)


        self._asclient = rospy.ServiceProxy('get_metrics', GetMetrics)

        self.action_trigger = False
        self.ac_fb = SingleRowExecutionFeedback()
        self.ac_result = SingleRowExecutionResult()
        self._as = actionlib.SimpleActionServer("execute_single_row", SingleRowExecutionAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

        rospy.Timer(rospy.Duration(0.1), self.publish)
        #rospy.spin()

    def initialize_test(self):
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
        if command == "START_TfEST":
            self.initialize_test()
            self.setPoses()
            self.rb.startBag()
        if command == "STOP_TEST":
            self.emergency_stop()

        if command == "NEXT_TEST":
            self.is_next_required = True

        self.command = command
        #self.setPose

    def execute_cb(self,goal):
        if self._as.is_preempt_requested():
            rospy.loginfo('Preempted')
            self._as.set_preempted()
            #self.emergency_stop()
        self.repetitions =  goal.repetitions
        rospy.logerr( " Repeat %s times" % str(self.repetitions))
        folder_name = "/home/jose/ros_ws/src/gr_navigation/gr_navigation_actions/simple_row_nav/"+ goal.row_id.data +"/"

        try:
            os.mkdir(folder_name)
        except OSError:
            print ("Creation of the directory failed")

        self.ac_fb = SingleRowExecutionFeedback()
        self.ac_result = SingleRowExecutionResult()
        self.action_trigger = True

        self.twist.linear.x = goal.linearspeed
        self.desired_speed = goal.linearspeed

        self.rb = utils.BagRecorder(record_topics = list_topics,
                                    desired_path = folder_name,
                                    enable_smach=False, start=False)
        self.distance = goal.cmd_distance
        self.voice_cb2(goal.command)
        while self.is_running():
            rospy.sleep(0.1)


        self.command = None

        srvmessage = GetMetricsRequest()
        srvmessage.file_name = "srvtest.txt"
        resp = self._asclient(srvmessage)
        self.ac_result.metrics = resp.metrics
        self._as.set_succeeded(self.ac_result)

    def voice_cb2(self,msg):
        command = parse_command(msg.data, False)
        print "command to execute ", command
        if command == "START_TEST":
            self.initialize_test()
            self.setPoses()
            self.rb.startBag()
        if command == "STOP_TEST":
            self.emergency_stop()

        if command == "NEXT_TEST":
            self.is_next_required = True

        self.command = command


    def swapPoses(self):
        tmp = self.startpose
        self.startpose = self.endpose
        self.endpose = tmp
        self.endpose[0] = -self.endpose[0]

    def setPoses(self):
        try:
            self.listener.waitForTransform('/odom', '/base_link', rospy.Time(0), rospy.Duration(2.0))
            (trans,rot) = self.listener.lookupTransform('/odom', '/base_link', rospy.Time(0))
            #print type(trans),rot
            self.startpose = [trans[0], trans[1]]
            #print self.startpose
            endpose = PoseStamped()
            endpose.header.frame_id = "base_link"
            endpose.pose.orientation.w = 1.0
            if self.forward:
                endpose.pose.position.x = self.distance
            else:
                endpose.pose.position.x = -self.distance
            p_end = self.listener.transformPose("odom", endpose)
            self.endpose = [p_end.pose.position.x, p_end.pose.position.y]
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            sys.exit()#continue


    def odom_cb(self, msg):
        self.currentpose[0] = msg.pose.pose.position.x
        self.currentpose[1] = msg.pose.pose.position.y
        self.start = True

    def is_running(self):
        return self.current_motions < self.repetitions

    def publish(self, event):
        if self.command is None:
            rospy.logwarn_throttle(4, "waiting for voice command")
            return

        self.pub.publish(self.twist)
        #print math.sqrt(math.pow(self.endpose[0] - self.currentpose[0],2) + math.pow(self.endpose[1] - self.currentpose[1],2))

        if self.is_next_required:
            self.change_direction()
        else:
            rospy.loginfo_throttle(5,math.sqrt(math.pow(self.endpose[0] - self.currentpose[0],2) + math.pow(self.endpose[1] - self.currentpose[1],2)))
            if math.sqrt(math.pow(self.endpose[0] - self.currentpose[0],2) + math.pow(self.endpose[1] - self.currentpose[1],2)) < 0.5:
                self.change_direction()


    def change_direction(self):

        #self.twist.linear.x = -self.twist.linear.x
        self.forward = not self.forward
        print ("change direction forward ", self.forward)
        if self.forward:
            self.twist.linear.x = self.desired_speed
        else:
            self.twist.linear.x = -0.25

        self.current_motions = self.current_motions + 1

        if self.action_trigger:
            rospy.logwarn("Publish fb")
            self.ac_fb.sequence = self.current_motions
            self._as.publish_feedback(self.ac_fb)

        if not self.is_next_required:
            self.setPoses()
        else:
            rospy.logerr("TODO Add turn around functionality")
            self.swapPoses()

        self.is_next_required = False

        if self.forward and self.is_running():
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
        self.current_motions = self.repetitions +1
        #self.as.set_aborted(self.ac_result)
        #self.initializeTests()

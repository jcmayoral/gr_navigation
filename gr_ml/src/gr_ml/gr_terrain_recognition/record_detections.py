#!/usr/bin/python
import rospy
import tf2_ros
import tf2_geometry_msgs
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import Header
import time
import os

class DetectionRecorder:
    def __init__(self, folder="measured_example" ):
        rospy.init_node("record_detection")
        self.folder = "measured_"+folder
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/pcl_gpu_tools/detected_objects", PoseArray, self.pose_cb)
        self.publisher = rospy.Publisher("/velodyne_points", PointCloud2)
        self.message_processed = True
        self.index = 1
        self.create_folder()
        time.sleep(2)


    def create_folder(self):
        #path = os.getcwd()
        try:
            os.mkdir(self.folder)
        except OSError:
            print ("Creation of the directory %s failed" % self.folder)
        else:
            print ("Successfully created the directory %s " % self.folder)


    def publish_gt(self, msg):
        self.publisher.publish(msg)
        self.message_processed = False

    def pose_cb(self,msg):
        stamp = str(msg.header.stamp.to_sec())
        frame_id = msg.header.frame_id
        default_class = "object"

        target_frame = "velodyne"

        header = ",".join(["x", "y", "timestamp", "class", "frame_id"])
        f = open(os.path.join(self.folder, str(self.folder) + str(self.index)),"w")
        f.write(header + "\n")


        for p in msg.poses:
            transform = self.tf_buffer.lookup_transform(target_frame,
                                           frame_id, #source frame
                                           rospy.Time(0), #get the tf at first available time
                                           rospy.Duration(0.1)) #wait for 1 second

            pose_stamped = PoseStamped()
            pose_stamped.header = msg.header
            pose_stamped.pose = p
            pose_transformed = tf2_geometry_msgs.do_transform_pose(pose_stamped, transform)
            info = ",".join([str(pose_transformed.pose.position.x), str(pose_transformed.pose.position.y), stamp, default_class, target_frame])
            f.write(info+"\n")
        f.close()
        self.message_processed = True
        self.index +=1

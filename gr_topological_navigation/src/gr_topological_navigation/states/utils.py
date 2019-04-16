import smach
import rosbag
import threading
import rospy

class BagRecorder(smach.State):
    def __init__(self, record_topics=dict()):
        self.is_finished = False
        self.is_bag_started = False
        self.lock = threading.Lock()

        for topic, type in record_topics.iteritems():
            rospy.loginfo("Loading ," + topic)
            rospy.Subscriber(topic, type, self.mainCB, topic, queue_size=300)

        self.startBag()
        smach.State.__init__(self,
                             outcomes=['RECORD_STARTED','END_RECORD'],
                             input_keys=['counter_in', 'shared_string', 'restart_requested'],
                             output_keys=['counter_out', 'restart_requested_out'])
        rospy.loginfo("Initializing")

    def startBag(self):
        self.bag = rosbag.Bag('starter.bag', 'w')
        self.is_finished = False
        self.is_bag_started= True
        rospy.sleep(5)

    def restart(self, root_name, bag_id):
        print ("NEW FILE")
        rospy.sleep(0.5)
        self.close()
        self.is_finished = True # Filter error of cb when file is already closed
        self.bag = rosbag.Bag(root_name + bag_id +'.bag', 'w')
        #userdata.counter_out = userdata.counter_in + 1
        self.is_bag_started = True
        rospy.sleep(0.5)
        self.is_finished = False # End Filter

    def execute(self, userdata):
        if userdata.restart_requested:
            self.restart(userdata.shared_string , str(userdata.counter_in))
            userdata.restart_requested_out = True
            return "RECORD_STARTED"


        if userdata.restart_requested is None:
            print ("EXITING")
            self.close()
            return "END_RECORD"

    def writeToBag(self,topic, msgs):
        self.lock.acquire()
        if not self.is_finished and self.is_bag_started:
            rospy.loginfo(topic)
            self.bag.write(topic, msgs)
        self.lock.release()

    def mainCB(self,msg, topic_name):
        self.writeToBag(topic_name, msg)

    def close(self):
        rospy.loginfo("Closing Bag File")
        self.is_bag_started = False
        self.is_finished = True
        self.bag.close()

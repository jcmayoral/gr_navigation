#!/usr/bin/python
from gr_topological_navigation.states.utils import BagRecorder
from std_msgs.msg import Bool, Empty
import rospy

if __name__ == '__main__':
    rospy.init_node("bag_tester")
    topics_dict = dict()
    topics_dict["test"] = Bool
    topics_dict["test_empty"] = Empty

    bag_recorder = BagRecorder(topics_dict)

    pub_1 = rospy.Publisher("test", Bool, queue_size =1)
    pub_2 = rospy.Publisher("test_empty", Empty, queue_size =5)

    value = True

    while not rospy.is_shutdown():
        value = not value
        pub_1.publish(Bool(value))
        pub_2.publish(Empty())

    bag_recorder.close()

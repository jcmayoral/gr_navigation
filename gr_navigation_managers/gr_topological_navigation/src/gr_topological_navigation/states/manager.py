import smach
import rospy
from std_msgs.msg import Empty

class Manager(smach.State):
    def __init__(self):
        ## HACK:
        self.rsub = rospy.Publisher("end_motion", Empty)
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH_REQUEST'],
                             input_keys=['counter_in', 'restart_requested', 'stop_requested', 'nodes_to_go', 'execution_requested' ],
                             output_keys=['counter_out', 'restart_requested_out', 'stop_requested_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        if userdata.execution_requested:
            rospy.logerr("execution requested")
            userdata.counter_out = 0
            #TODO find a way to stop
            userdata.stop_requested_out = False
            userdata.restart_requested_out = True
            return "SETUP_DONE"
        if userdata.stop_requested:
            rospy.logerr("stop requested")
            userdata.stop_requested_out = True
            self.rsub.publish(Empty())
            return "FINISH_REQUEST"
        if userdata.nodes_to_go != 0: # TODO Automatic File Indexing
            rospy.logerr("continue")
            userdata.counter_out = userdata.counter_in +1
            userdata.restart_requested_out = True
            return 'SETUP_DONE'
        else:
            rospy.logerr("finish")
            self.rsub.publish(Empty())
            userdata.stop_requested_out = True
            return 'FINISH_REQUEST'

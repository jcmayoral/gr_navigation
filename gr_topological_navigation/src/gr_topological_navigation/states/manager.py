import smach
import rospy

class Manager(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['SETUP_DONE', 'FINISH_REQUEST'],
                             input_keys=['counter_in', 'restart_requested', 'stop_requested'],
                             output_keys=['counter_out', 'restart_requested_out', 'stop_requested_out'])

    def execute(self, userdata):
        rospy.loginfo('Executing SETUP')
        rospy.sleep(0.5)
        if userdata.stop_requested:
            userdata.stop_requested_out = True
            return "FINISH_REQUEST"
        if userdata.counter_in < 5: # TODO Automatic File Indexing
            userdata.counter_out = userdata.counter_in +1
            userdata.restart_requested_out = True
            return 'SETUP_DONE'
        else:
            userdata.stop_requested_out = True
            return 'FINISH_REQUEST'

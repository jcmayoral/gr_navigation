#!/usr/bin/python
import rosbag
import rospy
import smach
from smach_ros import IntrospectionServer, MonitorState
import random
import sys
import tf

from gr_topological_navigation.states.utils import BagRecorder
from gr_topological_navigation.states.functions import update_cb
from gr_topological_navigation.states.manager import Manager
from gr_topological_navigation.states.topological_planner import TopologicalPlanner
from std_msgs.msg import Time


if __name__ == '__main__':
    rospy.init_node("full_coverage_plan")

    sm = smach.StateMachine(['succeeded','aborted','preempted','END_SM'])
    sm.userdata.sm_counter = 0
    sm.userdata.nodes_to_go = 100
    sm.userdata.bag_family = "topological_testing_"
    sm.userdata.restart_requested = True # This flag restart the cycle
    sm.userdata.stop_requested = False # This flag stops the recorder
    sm.userdata.execution_requested = False

    with sm:
        smach.StateMachine.add('IDLE', MonitorState("/update_map", Time, update_cb, max_checks=1, output_keys=['execution_requested']),
                            transitions={'invalid':'END_SM', 'valid':'SETUP', 'preempted':'SETUP'},
                            remapping={'execution_requested':'execution_requested'})
        smach.StateMachine.add('SETUP', Manager(),
                            transitions={'SETUP_DONE':'CONFIG_WRITER', 'FINISH_REQUEST': 'IDLE'},
                            remapping={'counter_in':'sm_counter',
                                'counter_out':'sm_counter',
                                'restart_requested_out':'restart_requested',
                                'stop_requested_out':'stop_requested'})
        #TODO provide topic and types somehow
        smach.StateMachine.add('CONFIG_WRITER', BagRecorder(),
                            transitions={'RECORD_STARTED':'TRIGGER_MOVE', 'END_RECORD': 'END_SM'},
                            remapping={'counter_in':'sm_counter',
                                'counter_out':'sm_counter',
                                'shared_string':'bag_family',
                                'restart_requested_out':'restart_requested',
                                'stop_requested_out':'stop_requested'})
        smach.StateMachine.add('TRIGGER_MOVE', TopologicalPlanner(pointset= "wish_map_move_base"),
              transitions={'NODE_REACHED':'SETUP', 'ERROR_NAVIGATION': 'END_SM'},
              remapping={'restart_requested_out':'restart_requested',
                             'stop_requested_out':'stop_requested',
                             'execution_requested_out':'execution_requested'})

    #Instrospection
    sis = IntrospectionServer('bag_writer', sm, '/SM_ROOT')
    sis.start()
    # Execute the state machine
    outcome = sm.execute()
    # Wait for ctrl-c to stop the application
    sis.stop()

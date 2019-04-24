#!/usr/bin/python
import rosbag
import rospy
import smach
from smach_ros import IntrospectionServer
import random
import sys
import tf

from gr_topological_navigation.states.utils import BagRecorder
from gr_topological_navigation.states.manager import Manager
from gr_topological_navigation.states.topological_planner import TopologicalPlanner

rospy.init_node("my_collision_bag_recorder")

sm = smach.StateMachine(['succeeded','aborted','preempted','END_SM'])
sm.userdata.sm_counter = 0
sm.userdata.nodes_to_go = 100
sm.userdata.bag_family = "topological_testing_"
sm.userdata.restart_requested = True # This flag restart the cycle
sm.userdata.stop_requested = False # This flag stops the recorder

with sm:
    smach.StateMachine.add('SETUP', Manager(),
                       transitions={'SETUP_DONE':'CONFIG_WRITER', 'FINISH_REQUEST': 'END_SM'},
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

    smach.StateMachine.add('TRIGGER_MOVE', TopologicalPlanner(start_node = "node_0", pointset= "trash_map_5"),
                  transitions={'NODE_REACHED':'SETUP', 'ERROR_NAVIGATION': 'END_SM'},
                  remapping={'restart_requested_out':'restart_requested',
                             'stop_requested_out':'stop_requested'})

#bagRecord.close()
#Instrospection
sis = IntrospectionServer('bag_writer', sm, '/SM_ROOT')
sis.start()
# Execute the state machine
outcome = sm.execute()
# Wait for ctrl-c to stop the application
sis.stop()

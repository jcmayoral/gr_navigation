#!/usr/bin/python
import rospy
import actionlib
import smach
from smach_ros import SimpleActionState
import networkx as nx
import numpy as np
import tf
import matplotlib.pyplot as plt

#TODO
#from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#Removing strands and rasberry dependencies
from mongodb_store.message_store import MessageStoreProxy
from navigation_msgs.msg import TopologicalMap

from std_msgs.msg import String, Bool
from gr_topological_navigation.states.move_base_state import command_robot_to_node, move_base, sbpl_action_mode

class TopologicalPlanner(SimpleActionState):
    def __init__(self, gui=False, pointset="riseholme_bidirectional_sim"):
        self.msg_store = MessageStoreProxy(collection="topological_maps")
        current_edge_subscriber = rospy.Subscriber("/current_edge", String, self.current_edge_callback, queue_size=2)
        self.current_node = None
        current_edge_subscriber = rospy.Subscriber("/current_node", String, self.current_node_callback, queue_size=2)

        request_tool_pub = rospy.Publisher("cut_grass", Bool, queue_size =1)

        #TODO ask for the parameter
        self.gui = gui
        self.pointset = pointset
        self.topological_plan = None
        self.nodes_poses = None

        #get_topological_map reset_graph
        self.get_map()

        self.is_task_initialized = False

        self.visited_edges = list()

        #Smach State Constructor
        SimpleActionState.__init__(self, "sbpl_action", MoveBaseAction,
                         outcomes=['NODE_REACHED','ERROR_NAVIGATION'], goal_cb = self.goal_cb,
                         result_cb = self.result_cb,
                         input_keys=['next_transition', 'nodes_to_go','execution_requested'],
                         output_keys=['restart_requested_out', 'stop_requested_out','next_transition', 'nodes_to_go', 'execution_requested_out'])

    def current_node_callback(self, node):
        if node.data == "none":
            return
        self.current_node = node.data

    def reset_graph(self,map):
        #generate_full_coverage_plan
        self.create_graph(map)
        self.generate_full_coverage_plan()

    def get_map(self):
        #get_topological_map = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        #msg = GetTopologicalMapRequest()
        #msg.pointset = self.pointset
        #map = get_topological_map(msg)
        query = self.msg_store.query_named("wish_map_move_base", TopologicalMap._type)
        map = query[0]
        print query[1]
        self.reset_graph(map)

    #Getting FB form topological navigation
    def current_edge_callback(self, edge_raw):

        if not self.is_task_initialized:
            return

        if edge_raw.data == 'none':
            return
        #remove map-name
        edge = edge_raw.data.split('--')[0]
        start = edge.split('_')[0]
        end = edge.split('_')[1]
        #rospy.loginfo("visiting edge %s", edge)

        if edge not in self.visited_edges:
            rospy.loginfo("visiting edge %s", edge)
            self.visited_edges.append(edge)
            self.visited_edges.append(end+'_'+start)
        else:
            rospy.logerr("visiting visited edge %s", edge)

    def reset(self):
        self.is_task_initialized = False
        self.visited_edges = list()
        self.get_map()

    def calculate_weight(self,a,b):
        return b[2]-a[2]


    def create_graph(self, map):
        self.networkx_graph = nx.Graph()
        self.nodes_poses = dict()

        for n in map.nodes:
            #assuming 2d map just yaw matters
            quaternion = (n.pose.orientation.x,
                          n.pose.orientation.y,
                          n.pose.orientation.z,
                          n.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)[2]
            self.networkx_graph.add_node(n.name)
            self.nodes_poses[n.name] =(n.pose.position.x, n.pose.position.y, euler)
            for e in n.edges:
                if not self.networkx_graph.has_edge(e.node, n.name) and  not self.networkx_graph.has_edge(n.name, e.node):
                    self.networkx_graph.add_edge(n.name, e.node, edge_id=e.edge_id)

        rospy.loginfo("%d Nodes found", self.networkx_graph.number_of_nodes())
        rospy.loginfo("%d Edges found", self.networkx_graph.number_of_edges())

        if self.gui:
            nx.draw(self.networkx_graph, pos = self.nodes_poses, with_labels=True, font_weight='bold')
            nx.draw_networkx_edge_labels(self.networkx_graph, self.nodes_poses, font_color='red')
            #nx.draw_networkx_edges(self.networkx_graph, pos = nx.spring_layout(self.networkx_graph))
            plt.show(False)

    #TODO
    #Add distance weight to the edges
    def generate_full_coverage_plan(self):
        rospy.logwarn("Generating Full Coverage Plan")
        #bfs_edges Not recommended
        """
        if self.networkx_graph.has_node(self.current_node):
            rospy.logerr("HELP")
            print "A Star",  nx.astar_path(self.networkx_graph, source=self.current_node, target="end_node")
            #self.topological_plan = list(nx.edge_dfs(self.networkx_graph, orientation='original'))
            self.topological_plan = nx.astar_path(self.networkx_graph, source= self.current_node, target="end_nodeend
            return
        """
        #print "A Star",  nx.astar_path(self.networkx_graph, source="start_node", target="end_node")
        self.topological_plan = nx.astar_path(self.networkx_graph, source="start_node", target="end_node")
        #self.topological_plan = list(nx.edge_dfs(self.networkx_graph, source=self.current_node, orientation='inverse'))
        #self.topological_plan = list(nx.dfs_tree(self.networkx_graph, source=self.start_node))
        #self.topological_plan = list(nx.dfs_preorder_nodes(self.networkx_graph, source=self.start_node))
        #self.start_node = self.topological_plan[0][0]
        print self.topological_plan

    def go_to_source(self):
        rospy.loginfo("Robot going to start node %s", self.topological_plan[0])
        sbpl_action_mode(self.nodes_poses[self.topological_plan[0]])
        #self.current_node = self.start_node
        self.is_task_initialized = True
        #self.generate_full_coverage_plan()

    def get_next_transition(self):
        if self.topological_plan is None:
            rospy.logerr("Topological Plan not initialized")
            return None
        if len(self.topological_plan) == 0:
            rospy.logerr("Next transition not found")
            return None

        next_edge = self.topological_plan.pop(0)
        print "next edge", next_edge
        #TODO find a better condition
        #for example analyzing path before proceed....
        # possible at move_base_flex
        angle = np.fabs(self.get_orientation(next_edge))
        while angle == 0 or  angle == 180:
            rospy.logwarn("skipping region")
            if len(self.topological_plan) == 0:
                rospy.logerr("Topological plan size is zero returning last edge")
                return next_edge
            next_edge = self.topological_plan.pop(0)

        return next_edge

    def get_orientation (self, edge):
        #this is in map c
        p0 = self.nodes_poses[edge]
        if self.current_node is None:
            return -5000
        p1 = self.nodes_poses[self.current_node]
        print np.arctan2(p0[0]- p1[0], p0[1] - p1[1]) * 180 / np.pi
        return np.arctan2(p0[0]- p1[0], p0[1] - p1[1]) * 180 / np.pi

    def goal_cb(self, userdata, goal):
        if userdata.execution_requested:
            userdata.execution_requested_out = False
            self.reset()
            self.go_to_source()
        print "HERE"

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        next_transition = self.get_next_transition()
        userdata.next_transition = next_transition

        if next_transition is None: #this could happens
            pose = self.nodes_poses[self.current_node]
        else:
            pose = self.nodes_poses[next_transition]

        goal.target_pose.pose.position.x = pose[0]
        goal.target_pose.pose.position.y = pose[1]
        quaternion = tf.transformations.quaternion_from_euler(0,0,pose[2])
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        return goal

    def result_cb(self, userdata, status, results):
        if results:
            userdata.nodes_to_go = len(self.topological_plan)
            userdata.restart_requested_out = True
            self.is_task_initialized = False
            return "NODE_REACHED"
        else: #if anything fails stop
            userdata.stop_requested_out = True
            if self.topological_plan is None:
                return
            userdata.missing_edges = len(self.topological_plan)
            return "ERROR_NAVIGATION"


    #TODO add more statistics
    def show_statistics(self):
        rospy.logwarn("%d nodes still missing", len(self.topological_plan))

#!/usr/bin/python
import rospy
import actionlib
import smach
from smach_ros import SimpleActionState
import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
from strands_navigation_msgs.srv import GetTopologicalMapRequest, GetTopologicalMap
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from std_msgs.msg import String, Bool
from gr_topological_navigation.states.move_base_state import command_robot_to_node, move_base

class TopologicalPlanner(SimpleActionState):
    def __init__(self, gui=False, start_node='cold_storage', pointset="riseholme_bidirectional_sim"):
        current_edge_subscriber = rospy.Subscriber("/current_edge", String, self.current_edge_callback, queue_size=2)
        request_tool_pub = rospy.Publisher("cut_grass", Bool, queue_size =1)

        #TODO ask for the parameter
        self.start_node = start_node
        self.gui = gui
        self.pointset = pointset
        self.topological_plan = None
        self.nodes_poses = None

        #get_topological_map reset_graph
        self.get_map()

        self.is_task_initialized = False

        self.visited_edges = list()
        self.current_node = "start"

        #Smach State Constructor
        SimpleActionState.__init__(self, "topological_navigation", GotoNodeAction,
                         outcomes=['NODE_REACHED','ERROR_NAVIGATION'], goal_cb = self.goal_cb,
                         result_cb = self.result_cb,
                         input_keys=['next_transition', 'nodes_to_go','execution_requested'],
                         output_keys=['restart_requested_out', 'stop_requested_out','next_transition', 'nodes_to_go', 'execution_requested_out'])

    def reset_graph(self,map):
        #generate_full_coverage_plan
        self.create_graph(map)
        self.generate_full_coverage_plan()

    def get_map(self):
        get_topological_map = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        msg = GetTopologicalMapRequest()
        msg.pointset = self.pointset
        map = get_topological_map(msg)
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

    def create_graph(self, map):
        self.networkx_graph = nx.Graph()
        self.nodes_poses = dict()

        for n in map.map.nodes:
            self.networkx_graph.add_node(n.name)
            self.nodes_poses[n.name] =(n.pose.position.x, n.pose.position.y)
            for e in n.edges:
                if not self.networkx_graph.has_edge(e.node, n.name) and  not self.networkx_graph.has_edge(n.name, e.node):
                    self.networkx_graph.add_edge(n.name, e.node, edge_id=e.edge_id)

        rospy.loginfo("%d Nodes found", self.networkx_graph.number_of_nodes())
        rospy.loginfo("%d Edges found", self.networkx_graph.number_of_edges())

        if self.gui:
            nx.draw(self.networkx_graph, pos = nodes_poses, with_labels=True, font_weight='bold')
            nx.draw_networkx_edge_labels(self.networkx_graph, nodes_poses, font_color='red')
            #nx.draw_networkx_edges(self.networkx_graph, pos = nx.spring_layout(self.networkx_graph))
            plt.show(False)

    #TODO
    #Add distance weight to the edges
    def generate_full_coverage_plan(self):
        rospy.logwarn("Generating Full Coverage Plan")
        #bfs_edges Not recommended
        self.topological_plan = list(nx.edge_dfs(self.networkx_graph, source=self.start_node, orientation='reverse'))
        #self.topological_plan = list(nx.dfs_tree(self.networkx_graph, source=self.start_node))
        #self.topological_plan = list(nx.dfs_preorder_nodes(self.networkx_graph, source=self.start_node))

    def go_to_source(self):
        rospy.loginfo("Robot going to start node %s", self.start_node)
        if self.nodes_poses is not None:
            rospy.logerr(self.nodes_poses[self.start_node])
        move_base(self.nodes_poses[self.start_node][0], self.nodes_poses[self.start_node][1])
        self.current_node = self.start_node
        self.is_task_initialized = True

    def get_next_transition(self):
        if self.topological_plan is None:
            rospy.logerr("Topological Plan not initialized")
            return None
        if len(self.topological_plan) == 0:
            rospy.logerr("Next transition not found")
            return None

        next_edge = self.topological_plan.pop(0)
        #TODO find a better condition
        #for example analyzing path before proceed....
        # possible at move_base_flex
        while np.fabs(self.get_orientation(next_edge)) == 90:
            rospy.logwarn("skipping region")
            if len(self.topological_plan) == 0:
                rospy.logerr("Topological plan size is zero returning last edge")
                return next_edge
            next_edge = self.topological_plan.pop(0)


        return next_edge

    def get_orientation (self, edge):
        #this is in map c
        p0 = self.nodes_poses[edge[0]]
        p1 = self.nodes_poses[edge[1]]

        dx = p1[0] - p0[0]
        dy = p1[1] - p0[0]
        print "edge ", edge, "angle ", np.arctan2(dy, dx) * 180 / np.pi
        return np.arctan2(dy, dx) * 180 / np.pi

    def goal_cb(self, userdata, goal):

        if userdata.execution_requested: #not implemented yet
            self.reset()
            userdata.execution_requested_out = False
            self.go_to_source()

        if not self.is_task_initialized:
            self.go_to_source()

        next_transition = self.get_next_transition()
        userdata.next_transition = next_transition
        if next_transition is None: #this could happens
            goal.target = self.current_node
        else:
            goal.target = next_transition[1]

        goal.no_orientation = True
        return goal

    def result_cb(self, userdata, status, results):
        if results:
            userdata.nodes_to_go = len(self.topological_plan)
            userdata.restart_requested_out = True
            self.current_node = userdata.next_transition
            return "NODE_REACHED"
        else: #if anything fails stop
            userdata.missing_edges = len(self.topological_plan)
            userdata.stop_requested_out = True
            return "ERROR_NAVIGATION"


    #TODO add more statistics
    def show_statistics(self):
        rospy.logwarn("%d nodes still missing", len(self.topological_plan))
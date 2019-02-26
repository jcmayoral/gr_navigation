#!/usr/bin/python
import rospy
import actionlib
import networkx as nx
import matplotlib.pyplot as plt
from strands_navigation_msgs.srv import GetTopologicalMapRequest, GetTopologicalMap
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal

class TopologicalPlanner:
    def __init__(self, gui=False):
        rospy.init_node("topological_full_coverage_planner")
        get_topological_map = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        msg = GetTopologicalMapRequest()
        #TODO ask for the parameter
        msg.pointset = "simulation_map"
        #TODO what to do in case of failures
        map = get_topological_map(msg)
        #Topological Map parsed to a networkx Graph so functions can be used
        self.networkx_graph = nx.Graph()
        self.gui = gui
        self.topological_plan = None
        self.create_graph(map)
        self.action_client = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
        rospy.loginfo("Waiting for Action Server /topological_navigation")
        self.action_client.wait_for_server()
        rospy.loginfo("Action Server Found")

    def create_graph(self, map):
        for n in map.map.nodes:
            self.networkx_graph.add_node(n.name)
            for e in n.edges:
                self.networkx_graph.add_edge(n.name, e.node, edge_id=e.edge_id)

        rospy.loginfo("%d Nodes found", self.networkx_graph.number_of_nodes())
        rospy.loginfo("%d Edges found", self.networkx_graph.number_of_edges())

        if self.gui:
            nx.draw(self.networkx_graph, with_labels=True, font_weight='bold')
            plt.show()

    #TODO ask for current edge or node
    def generate_full_coverage_plan(self):
        self.topological_plan = list(nx.edge_dfs(self.networkx_graph))

    def get_next_transition(self):
        if self.topological_plan is None:
            rospy.logerr("Topological Plan not initialized")
            return False
        if len(self.topological_plan) == 0:
            return False
        next_node = self.topological_plan.pop(0)
        return next_node

    def command_robot_to_node(self,node_id):
        rospy.loginfo("Commanding robot to %s", node_id)
        navgoal = GotoNodeGoal()
        navgoal.target = node_id
        navgoal.no_orientation = True
        self.action_client.send_goal(navgoal)
        self.action_client.wait_for_result()
        rospy.logwarn("%d nodes still missing", len(self.topological_plan))
        rospy.loginfo("Is node %s reached? %r", node_id,  self.action_client.get_result())  # A FibonacciResul

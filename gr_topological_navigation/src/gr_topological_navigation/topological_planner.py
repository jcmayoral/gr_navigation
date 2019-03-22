#!/usr/bin/python
import rospy
import actionlib
import networkx as nx
import matplotlib.pyplot as plt
from strands_navigation_msgs.srv import GetTopologicalMapRequest, GetTopologicalMap
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from std_msgs.msg import String

class TopologicalPlanner:
    def __init__(self, gui=False, start_node='cold_storage'):
        rospy.init_node("topological_full_coverage_planner")
        get_topological_map = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        current_edge_subscriber = rospy.Subscriber("/current_edge", String, self.current_edge_callback, queue_size=2)
        msg = GetTopologicalMapRequest()
        #TODO ask for the parameter
        msg.pointset = "simulation_map"
        #TODO ask for the parameter
        self.start_node = start_node
        #TODO what to do in case of failures
        map = get_topological_map(msg)
        #Topological Map parsed to a networkx Graph so functions can be used
        self.networkx_graph = nx.Graph()
        self.gui = gui
        self.topological_plan = None
        self.create_graph(map)
        self.visited_edges = list()
        self.current_node = "start"
        self.action_client = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
        rospy.loginfo("Waiting for Action Server /topological_navigation")
        self.action_client.wait_for_server()
        rospy.loginfo("Action Server Found")

    #Getting FB form topological navigation
    def current_edge_callback(self, edge_raw):
        if edge_raw.data == 'none':
            return
        #remove map-name
        edge = edge_raw.data.split('--')[0]
        start = edge_raw.data.split('_')[0]
        end = edge_raw.data.split('_')[1]
        rospy.loginfo("visiting edge %s", edge)

        if edge not in self.visited_edges:
            self.visited_edges.append(edge)
            self.visited_edges.append(end+'_'+start)

    def create_graph(self, map):
        for n in map.map.nodes:
            self.networkx_graph.add_node(n.name)
            for e in n.edges:
                self.networkx_graph.add_edge(n.name, e.node, edge_id=e.edge_id, weight=1)

        rospy.loginfo("%d Nodes found", self.networkx_graph.number_of_nodes())
        rospy.loginfo("%d Edges found", self.networkx_graph.number_of_edges())

        if self.gui:
            nx.draw(self.networkx_graph, with_labels=True, font_weight='bold')
            plt.show()

    #TODO
    #Add distance weight to the edges
    def generate_full_coverage_plan(self):
        #bfs_edges Not recommended
        self.topological_plan = list(nx.edge_dfs(self.networkx_graph, source=self.start_node, orientation="reverse"))

    def go_to_source(self):
        rospy.loginfo("Robot going to start node %s", self.start_node)
        self.command_robot_to_node(self.start_node, no_orientation=False)

    def get_next_transition(self):
        if self.topological_plan is None:
            rospy.logerr("Topological Plan not initialized")
            return False
        if len(self.topological_plan) == 0:
            rospy.loginfo("Full Coverage Topological Navigation is completed")
            return False

        next_edge = self.topological_plan.pop(0)
        suggested_edge = str(next_edge[0]+"_"+next_edge[1])
        inverted_edge = str(next_edge[1]+"_"+next_edge[0])

        if len(self.visited_edges)> 0:
            if suggested_edge in self.visited_edges:
                rospy.logwarn("Revisiting edge %s", suggested_edge)
            if inverted_edge in self.visited_edges:
                rospy.logwarn("Revisiting edge %s", inverted_edge)

        return next_edge

    def command_robot_to_node(self,node_id, no_orientation=True):
        rospy.loginfo("Commanding robot to %s", node_id)
        navgoal = GotoNodeGoal()
        navgoal.target = node_id
        navgoal.no_orientation = no_orientation
        self.action_client.send_goal(navgoal)
        self.action_client.wait_for_result()
        rospy.loginfo("Is node %s reached? %r", node_id,  self.action_client.get_result())
        self.current_node = node_id


    #TODO add more statistics
    def show_statistics(self):
        rospy.logwarn("%d edges still missing", len(self.topological_plan))

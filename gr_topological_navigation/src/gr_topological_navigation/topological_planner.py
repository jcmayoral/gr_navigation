#!/usr/bin/python
import rospy
import networkx as nx
from strands_navigation_msgs.srv import GetTopologicalMapRequest, GetTopologicalMap
import matplotlib.pyplot as plt

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
        self.create_graph(map)

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

    #TODO ask for current edge
    def get_plan_list(self):
        return list(nx.edge_dfs(self.networkx_graph))

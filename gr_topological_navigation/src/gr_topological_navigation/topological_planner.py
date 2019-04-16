#!/usr/bin/python
import rospy
import actionlib
import networkx as nx
import matplotlib.pyplot as plt
from strands_navigation_msgs.srv import GetTopologicalMapRequest, GetTopologicalMap
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from std_msgs.msg import String, Bool

class TopologicalPlanner:
    def __init__(self, gui=True, start_node='cold_storage'):
        get_topological_map = rospy.ServiceProxy("/topological_map_publisher/get_topological_map", GetTopologicalMap)
        rospy.init_node("topological_full_coverage_planner")
        current_edge_subscriber = rospy.Subscriber("/current_edge", String, self.current_edge_callback, queue_size=2)
        request_tool_pub = rospy.Publisher("cut_grass", Bool, queue_size =1)
        msg = GetTopologicalMapRequest()
        #TODO ask for the parameter
        msg.pointset = "riseholme_bidirectional_sim"
        #TODO ask for the parameter
        self.start_node = start_node
        #TODO what to do in case of failures
        map = get_topological_map(msg)
        #Topological Map parsed to a networkx Graph so functions can be used
        self.networkx_graph = nx.Graph()
        self.gui = gui
        self.is_task_initialized = False
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


    def create_graph(self, map):
        nodes_poses = dict()

        for n in map.map.nodes:
            self.networkx_graph.add_node(n.name)
            nodes_poses[n.name] =(n.pose.position.x, n.pose.position.y)
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
        #bfs_edges Not recommended
        self.topological_plan = list(nx.edge_dfs(self.networkx_graph, source=self.start_node, orientation='reverse'))
        #self.topological_plan = list(nx.dfs_tree(self.networkx_graph, source=self.start_node))
        #self.topological_plan = list(nx.dfs_preorder_nodes(self.networkx_graph, source=self.start_node))

    def go_to_source(self):
        rospy.loginfo("Robot going to start node %s", self.start_node)
        self.command_robot_to_node(self.start_node, no_orientation=False)
        self.is_task_initialized = True

    def get_next_transition(self):
        if self.topological_plan is None:
            rospy.logerr("Topological Plan not initialized")
            return False
        if len(self.topological_plan) == 0:
            rospy.loginfo("Full Coverage Topological Navigation is completed")
            return False

        next_edge = self.topological_plan.pop(0)
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
        rospy.logwarn("%d nodes still missing", len(self.topological_plan))

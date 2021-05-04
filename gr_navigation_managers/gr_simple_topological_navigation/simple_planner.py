from visualization_msgs.msg import MarkerArray, Marker
import rospy
import networkx as nx
import tf
import matplotlib.pyplot as plt
from gr_topological_navigation.states.move_base_state import move_base as move_base_server
import actionlib
from gr_action_msgs.msg import GRNavigationAction, GRNavigationActionGoal, GRNavigationActionResult

class SimpleTopoPlanner:
    def __init__(self):
        self.temporal_map = None
        #self.map_sub = rospy.Subscriber("/current_topological_map", MarkerArray, self.map_cb)
        self._as = actionlib.SimpleActionServer("gr_simple_manager", GRNavigationAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()

    def execute_cb(self, goal):
        result = GRNavigationActionResult()
        result.result.suceeded = False

        if self.create_graph(goal.plan.markers):
            print "MY PLAN "
            self.plan = self.get_topological_plan()
            #TODO SET TRIGGER
            self.execute_plan()
            result.result.suceeded = True
        #NOT so sure why this crashes
        #self._as.set_succeeded(result)
        self._as.set_succeeded()


    def map_cb(self, map):
        rospy.loginfo("new map arriving")
        if self.create_graph(map.markers):
            print "MY PLAN "
            self.plan = self.get_topological_plan()
            #TODO SET TRIGGER
            self.execute_plan()

    def execute_plan(self):
        print (self.plan)
        for node in self.plan:
            print node
            print self.nodes_poses[node]
            print move_base_server(self.nodes_poses[node], "sbpl_action")

    def get_topological_plan(self):
        return nx.astar_path(self.networkx_graph, source="start_node", target="end_node")


    def create_graph(self, amap):
        self.networkx_graph = nx.Graph()
        self.nodes_poses = dict()
        n_poses = dict()

        for n in amap:
            if n.action == Marker.DELETE or n.action == Marker.DELETEALL:
                print ("skip delete marker and reset temporal MAP")
                self.temporal_map = None
                return False
            if n.ns == "edges":
                startnode, endnode = n.text.split("::")
                if not self.networkx_graph.has_edge(startnode, endnode):
                    self.networkx_graph.add_edge(startnode, endnode, edge_id=n.text)
                continue


            quaternion = (n.pose.orientation.x,
                          n.pose.orientation.y,
                          n.pose.orientation.z,
                          n.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)[2]
            self.networkx_graph.add_node(n.text)
            n_poses[n.text] =[n.pose.position.x, n.pose.position.y]
            self.nodes_poses[n.text] =[n.pose.position.x, n.pose.position.y, euler]

        rospy.loginfo("%d Nodes found", self.networkx_graph.number_of_nodes())
        rospy.loginfo("%d Edges found", self.networkx_graph.number_of_edges())

        if False: #self.gui:
            nx.draw(self.networkx_graph, pos=n_poses,with_labels=True, font_weight='bold')
            nx.draw_networkx_edge_labels(self.networkx_graph, n_poses, font_color='red')
            nx.draw_networkx_edges(self.networkx_graph, pos = nx.spring_layout(self.networkx_graph))
            plt.show()
        return True


if __name__ == "__main__":
    rospy.init_node("simple_topoplanner")
    planner = SimpleTopoPlanner()
    rospy.spin()

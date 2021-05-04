from visualization_msgs.msg import MarkerArray, Marker
import rospy
import networkx as nx
import tf
import matplotlib.pyplot as plt

class SimpleTopoPlanner:
    def __init__(self):
        self.temporal_map = None
        self.map_sub = rospy.Subscriber("/current_topological_map", MarkerArray, self.map_cb)

    def map_cb(self, map):
        rospy.loginfo("new map arriving")
        #for node in map.markers:
        #    print (node.ns)
        #    print (node.text)
        self.temporal_map = map
        self.create_graph(map.markers)

    def create_graph(self, amap):
        self.networkx_graph = nx.Graph()
        self.nodes_poses = dict()
        n_poses = dict()

        for n in amap:
            if n.action == Marker.DELETE or n.action == Marker.DELETEALL:
                print "skip delete marker and reset temporal MAP"
                self.temporal_map = None
                return
            if n.ns == "edges":
                startnode, endnode = n.text.split("::")
                if not self.networkx_graph.has_edge(startnode, endnode):
                    self.networkx_graph.add_edge(startnode, endnode, edge_id=n.text)
                continue
            #print (" text ", n.text)
            #print (" id ", n.id)
            #assuming 2d map just yaw matters
            quaternion = (n.pose.orientation.x,
                          n.pose.orientation.y,
                          n.pose.orientation.z,
                          n.pose.orientation.w)
            euler = tf.transformations.euler_from_quaternion(quaternion)[2]
            self.networkx_graph.add_node(n.text)
            n_poses[n.text] =[n.pose.position.x, n.pose.position.y]

            self.nodes_poses[n.text] =[n.pose.position.x, n.pose.position.y, euler]

            #poses.append([n.pose.position.x, n.pose.position.y])
            if n.ns == "edges":
                print ("Abort")
                continue
                if not self.networkx_graph.has_edge(e.node, n.name) and  not self.networkx_graph.has_edge(n.name, e.node):
                    self.networkx_graph.add_edge(n.name, e.node, edge_id=e.edge_id)

        rospy.loginfo("%d Nodes found", self.networkx_graph.number_of_nodes())
        rospy.loginfo("%d Edges found", self.networkx_graph.number_of_edges())

        if True:#self.gui:
            print n_poses
            nx.draw(self.networkx_graph, pos=n_poses,with_labels=True, font_weight='bold')
            #nx.draw_networkx_edge_labels(self.networkx_graph, self.nodes_poses, font_color='red')
            nx.draw_networkx_edges(self.networkx_graph, pos = nx.spring_layout(self.networkx_graph))
            plt.show()


if __name__ == "__main__":
    rospy.init_node("simple_topoplanner")
    planner = SimpleTopoPlanner()
    rospy.spin()

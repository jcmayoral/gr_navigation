from visualization_msgs.msg import MarkerArray, Marker
import rospy
import networkx as nx
import numpy as np
import tf
import matplotlib.pyplot as plt
#from gr_topological_navigation.states.move_base_state import move_base2 as move_base_server
from gr_topological_navigation.states.move_base_state import polyfit_action_mode as polyfit_server
import actionlib
from gr_action_msgs.msg import GRNavigationAction, GRNavigationActionGoal, GRNavigationActionResult, GRNavigationFeedback, PolyFitRowAction, PolyFitRowResult
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
import time
import dynamic_reconfigure.client
from mongoutils import MongoManager
from safety_msgs.msg import ExecutionMetadata

class SimpleTopoPlanner:
    def __init__(self):
        self.temporal_map = None
        #self.map_sub = rospy.Subscriber("/current_topological_map", MarkerArray, self.map_cb)
        self._as = actionlib.SimpleActionServer("gr_simple_manager", GRNavigationAction, execute_cb=self.execute_cb, auto_start = False)
        #self._as = actionlib.SimpleActionServer("gr_simple_manager", PolyFitRowAction, execute_cb=self.execute_cb, auto_start = False)
        #self.action_client = actionlib.SimpleActionClient('polyfit_action', PolyFitRowAction)
        self.action_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.goal_received = False
        self.goal_finished = False
        self.mongo_utils = MongoManager()

        self.dynconf_client = dynamic_reconfigure.client.Client("topological_to_metric_converter", timeout=10, config_callback=self.config_callback)

        self._as.start()

    def config_callback(self,config):
        rospy.loginfo("Config set to ", config)

    def move_base_server(self, commands):
        rospy.loginfo("Waiting for Action Server ")
        self.action_client.wait_for_server()
        rospy.loginfo("Action Server Found sending "+ str(commands))
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = commands[0]
        goal.target_pose.pose.position.y = commands[1]

        quaternion = tf.transformations.quaternion_from_euler(0,0,commands[2])
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        self.action_client.send_goal(goal, done_cb = self.done_cb, feedback_cb=self.feedback_cb, active_cb=self.active_cb)
        #print "WAITING FOR RESULT"
        #action_client.wait_for_result()
        #print "RESULT GOTTEN "
        #self.action_client.get_status()
        #print "after "

        return #action_client.get_result()

    def active_cb(self):
        print "GOAL received"
        self.goal_received = True

    def feedback_cb(self, feedback):
        pass
        #print "FB ", feedback

    def done_cb(self, state, result):
        print "Goal finished with state ", state
        print "Goal finished with result ", result
        self.goal_finished = True

    def execute_cb(self, goal):
        result = GRNavigationActionResult()
        result.result.suceeded = False

        if self.create_graph(goal.plan.markers):
            print "MY PLAN from {} to {}".format(goal.start_node, goal.goal_node)
            self.plan = self.get_topological_plan(goal.start_node, goal.goal_node)
            self.startnode = goal.start_node
            self.goalnode = goal.goal_node
            #TODO SET TRIGGER
            if self.execute_plan(goal.mode,goal.span):
                result.result.suceeded = True
            else:
                result.result.suceeded = False
        #NOT so sure why this crashes
        #self._as.set_succeeded(result)
        if result.result.suceeded:
            self._as.set_succeeded()
            return
        self._as.set_aborted()


    def map_cb(self, map):
        rospy.loginfo("new map arriving")
        if self.create_graph(map.markers):
            print "MY PLAN "
            self.plan = self.get_topological_plan("start", "end")
            #TODO SET TRIGGER
            self.execute_plan()

    def waitMoveBase(self):
        while not self.goal_finished:
            if self._as.is_preempt_requested():
                print "Cancel received"
                self.action_client.cancel_all_goals()
                return False
            time.sleep(1)
        return True

    def execute_plan(self,mode, span=0):
        print ("THIS IS MY PLAN " ,self.plan)
        #print "MOVING TO START ", move_base_server(self.nodes_poses["start_node"], self.action_client)
        """
        #POLYFIT
        #for p in self.plan:
        poses = []

        for n in self.plan:
            p = self.nodes_poses[n]
            poses.append([p[0], p[1],p[2]])
        print ("poses", np.asarray(poses).shape)
        polyfit_server(np.asarray(poses), self.action_client)
        return
        """
        self.goal_received = False
        self.goal_finished = False

        fb = GRNavigationFeedback()
        if mode == 0:#GRNavigationAction.VISIT_ALL:
            for node in self.plan:
                exec_msg = ExecutionMetadata()
                self.goal_received = False
                self.goal_finished = False
                print "moving to " , node
                if node == self.startnode:
                    self.dynconf_client.update_configuration({"constrain_motion": False})
                else:
                    self.dynconf_client.update_configuration({"constrain_motion": True})
                #WAIT FOR MAP UPDATE
                time.sleep(1)

                self.move_base_server(self.nodes_poses[node])
                print  self.action_client.get_state()
                if not self.waitMoveBase():
                    return False
                else:
                    fb.reached_node.data = node
                    #print fb
                    self._as.publish_feedback(fb)
                self.mongo_utils.insert_in_collection(exec_msg)

            return True
        #print self.nodes_poses["start_node"]
        #print move_base_server(self.nodes_poses["start_node"], "sbpl_action")
        #priyynt self.nodes_poses["end_node"]
        #print move_base_server(self.nodes_poses["end_node"], "sbpl_action")
        elif mode == 1: #GRNavigationAction.JUST_END:
            goals = [self.startnode, self.goalnode]
            for node in goals:
                exec_msg = ExecutionMetadata()
                self.goal_received = False
                self.goal_finished = False

                print "moving to ", node

                if node == self.startnode:
                    self.dynconf_client.update_configuration({"constrain_motion": False})
                else:
                    self.dynconf_client.update_configuration({"constrain_motion": True})
                #WAIT FOR MAP UPDATE
                time.sleep(1)

                self.move_base_server(self.nodes_poses[node])
                if not self.waitMoveBase():
                    return False
                else:
                    #fb.feedback.reached_node = node
                    fb.reached_node.data = node
                    self._as.publish_feedback(fb)
                print "SAVE TO MONGO"
                self.mongo_utils.insert_in_collection(exec_msg)
            return True
        #LAST TO BE IMPLEMENTED
        elif mode == 2: #GRNavigationActionGoal.VISIT_SOME:
            for n in range(0,len(self.plan),span):
                exec_msg = ExecutionMetadata()

                self.goal_received = False
                self.goal_finished = False
                print "VISIT_SOME", self.plan[n]

                if self.plan[n] == self.startnode:
                    self.dynconf_client.update_configuration({"constrain_motion": False})
                else:
                    self.dynconf_client.update_configuration({"constrain_motion": True})

                #WAIT FOR MAP UPDATE
                time.sleep(1)

                self.move_base_server(self.nodes_poses[self.plan[n]])
                if not self.waitMoveBase():
                    return False
                else:
                    #fb.reached_node = node
                    print self.plan[n]
                    fb.reached_node.data = self.plan[n]
                    self._as.publish_feedback(fb)
                self.mongo_utils.insert_in_collection(exec_msg)
        else:
            rospy.logerr("ERROR")
            return False



    def get_topological_plan(self, startnode, goalnode):
        return nx.astar_path(self.networkx_graph, source=startnode, target=goalnode)


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

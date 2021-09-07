from visualization_msgs.msg import MarkerArray, Marker
import rospy
import networkx as nx
import numpy as np
import tf
import matplotlib.pyplot as plt
from gr_topological_navigation.states.move_base_state import polyfit_action_mode as polyfit_server
import actionlib
from gr_action_msgs.msg import GRNavigationAction, GRNavigationActionGoal, GRNavigationActionResult, GRNavigationFeedback, PolyFitRowAction, PolyFitRowResult
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray
import time
import dynamic_reconfigure.client
from mongoutils import MongoManager
from safety_msgs.msg import ExecutionMetadata
from actionlib_msgs.msg import GoalStatus
import yaml
from std_msgs.msg import Bool
import copy
from tool_utils import ToolInterface

class SimpleTopoPlanner:
    def __init__(self):
        self.temporal_map = None
        self.container_full = False
        #self.map_sub = rospy.Subscriber("/`current_topological_map`", MarkerArray, self.map_cb)
        self.map_sub = rospy.Subscriber("/container_full", Bool, self.container_cb)

        self._as = actionlib.SimpleActionServer("gr_simple_manager", GRNavigationAction, execute_cb=self.execute_cb, auto_start = False)
        #self._as = actionlib.SimpleActionServer("gr_simple_manager", PolyFitRowAction, execute_cb=self.execute_cb, auto_start = False)
        #self.action_client = actionlib.SimpleActionClient('polyfit_action', PolyFitRowAction)
        self.action_client = actionlib.SimpleActionClient("move_base_flex/move_base", MoveBaseAction)
        self.load_movebase_params()
        self.goal_received = False
        self.goal_finished = False
        self.mongo_utils = MongoManager()

        self.dynconf_client = dynamic_reconfigure.client.Client("topological_to_metric_converter", timeout=10, config_callback=self.config_callback)

        self._as.start()

    def container_cb(self, is_full):
        rospy.logwarn("CONTAINER SIGNAL "+ str(is_full))
        self.container_full = is_full.data

    def load_movebase_params(self):
        with open("mbf_config.yaml") as f:
            self.params = yaml.load(f.read(), Loader=yaml.Loader)

    def config_callback(self,config):
        pass
        #rospy.loginfo("Config se", config)

    def move_base_server(self, commands, nav_mode):
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

        rospy.logerr("NAV_MODE: " + nav_mode)
        goal.controller = self.params[nav_mode]["controller"]
        goal.planner = self.params[nav_mode]["planner"]

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        self.lastupdate_time = time.time()
        self.last_pose = None
        self.distance_covered = 0.0
        self.action_client.send_goal(goal, done_cb = self.done_cb, feedback_cb=self.feedback_cb, active_cb=self.active_cb)

        return #action_client.get_result()

    def active_cb(self):
        rospy.loginfo("GOAL received")
        self.goal_received = True

    def calc_distance(self,a,b):
        return np.sqrt(np.power(b[0]-a[0],2)+np.power(b[1]-a[1],2))

    def feedback_cb(self, feedback):
        self.dist2goal = feedback.dist_to_goal
        q = (feedback.current_pose.pose.orientation.x,
            feedback.current_pose.pose.orientation.y,
            feedback.current_pose.pose.orientation.z,
            feedback.current_pose.pose.orientation.w)
        yaw = tf.transformations.euler_from_quaternion(q)[2]
        bp = [feedback.current_pose.pose.position.x, feedback.current_pose.pose.position.y, yaw]

        if self.last_pose:
            self.distance_covered +=  self.calc_distance(self.last_pose, bp)
            #rospy.loginfo("Distance covered {} meters".format(self.distance_covered))
            self.lastupdate_time = time.time()

        self.last_pose = bp


    def done_cb(self, state, result):
        print "Goal finished with state ", state
        if state == 0:
            print "SUCCESS"
            self.distance_covered += self.dist2goal#self.calc_distance(self.goal[:2], self.last_pose)
        if state == 12:
            print "Collision"
        if state == 16:
            print "TF ERROR"
        if state == 17:
            print "internal Error"
        if state == 13:
            print "oscillation"
        if state == 10:
            print "failure"
        self.goal_finished = True

    def execute_cb(self, goal):
        result = GRNavigationActionResult()
        result.result.suceeded = False

        if self.create_graph(goal.plan.markers):
            rospy.loginfo("MY PLAN from {} to {}".format(goal.start_node, goal.goal_node))
            self.plan = self.get_topological_plan(goal.start_node, goal.goal_node)
            self.startnode = goal.start_node
            self.goalnode = goal.goal_node
            self.rowid = goal.row_id
            self.taskid = goal.task_id
            
            #Interface to tool TO BE TESTED
            self.tool_interface = ToolInterface(**self.params[self.taskid]["tool_info"])

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

    def waitMoveBase(self, flag = True):
        while not self.goal_finished:
            if self._as.is_preempt_requested():
                rospy.logerr("Cancel received")
                self.action_client.cancel_all_goals()
                return False
            if self.container_full and flag:
                rospy.logerr("container full... going to container location")
                self.action_client.cancel_all_goals()
                rospy.sleep(2)
                return False
            time.sleep(1)
        rospy.loginfo("MOTION suceeded")
        return True

    def go_to_unload_zone(self, current_goal):
        last_knownpose = copy.copy(self.last_pose)

        #GO TO CONTAINER
        if not self.perform_motion([self.params["CONTAINER"]["x"],
                             self.params["CONTAINER"]["y"],
                             self.params["CONTAINER"]["yaw"]],
                              "CONTAINER", False, "CONTAINER"):
            return False
        #GOING BACK TO LAST KNOW POSE
        if not self.perform_motion(last_knownpose, "LAST_POSE", False, "CONTAINER"):
            return False
        #GOING BACK TO LAST KNOW GOAL
        if not self.perform_motion(current_goal, "LAST_GOAL", True, "FREE_MOTION"):
            return False

        rospy.logerr("UNLOADING .... add signal")
        self.container_full = False
        
        #RESTART TOOL
        self.tool_interface.start()
        return True

    def perform_motion(self,goal, id, constrain_motion, mode):
        rospy.logwarn("MODE ")
        starttime = time.time()
        exec_msg = ExecutionMetadata()
        exec_msg.rowid = -1
        exec_msg.action = "GO_TO_"+id
        self.goal_received = False
        self.goal_finished = False
        rospy.logwarn("moving to " + id + " with coordinates " + str(goal))

        #TODO ADD OSM MAP STUFF
        self.dynconf_client.update_configuration({"constrain_motion": constrain_motion})
        #WAIT FOR MAP UPDAT
        time.sleep(3)


        #True/False decides controller type
        self.move_base_server(goal, mode)

        if not self.waitMoveBase(False):
            return False

        exec_msg.time_of_execution = time.time() - starttime
        exec_msg.covered_distance = self.distance_covered
        self.mongo_utils.insert_in_collection(exec_msg, self.taskid)
        return True



    def execute_plan(self,mode, span=0):
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
                starttime = time.time()
                exec_msg = ExecutionMetadata()

                exec_msg.rowid = self.rowid
                exec_msg.action = "RUN"
                self.goal_received = False
                self.goal_finished = False
                rospy.logwarn("moving to " + node)
                if node == self.startnode:
                    exec_msg.action = "FREE_MOTION"
                    self.dynconf_client.update_configuration({"constrain_motion": False})
                else:
                    self.dynconf_client.update_configuration({"constrain_motion": True})
                #WAIT FOR MAP UPDATE
                time.sleep(3)
                self.goal = self.nodes_poses[node]
                nav_mode = "FREE_MOTION" if exec_msg.action == "FREE_MOTION" else self.taskid
                self.move_base_server(self.goal, nav_mode)

                if not self.waitMoveBase():
                    return False
                else:
                    fb.reached_node.data = node
                    self._as.publish_feedback(fb)

                exec_msg.time_of_execution = time.time() - starttime
                exec_msg.covered_distance = self.distance_covered
                self.mongo_utils.insert_in_collection(exec_msg, self.taskid)

            return True
        elif mode == 1: #GRNavigationAction.JUST_END:
            goals = [self.startnode, self.goalnode]

            for node in goals:
                exec_msg = ExecutionMetadata()
                starttime = time.time()
                exec_msg.rowid = self.rowid
                exec_msg.taskid = self.taskid
                exec_msg.action = "RUN"

                self.goal_received = False
                self.goal_finished = False

                rospy.logwarn ("moving to " + node + " POSE " + str(self.nodes_poses[node]))

                if node == self.startnode:
                    exec_msg.action = "FREE_MOTION"
                    self.dynconf_client.update_configuration({"constrain_motion": False})
                else:
                    self.dynconf_client.update_configuration({"constrain_motion": True})
                #WAIT FOR MAP UPDATE
                time.sleep(3)
                self.goal =self.nodes_poses[node]
                nav_mode = "FREE_MOTION" if exec_msg.action == "FREE_MOTION" else self.taskid

                #START TOOL
                if nav_mode != "FREE_MOTION" and nav_mode != "CONTAINER":
                    self.tool_interface.start()
                
                self.move_base_server(self.goal, nav_mode)
                if not self.waitMoveBase():
                    self.tool_interface.stop()
                    if self.container_full:
                        if self.go_to_unload_zone(self.goal):
                            rospy.logwarn("Resuming execution")
                        else:
                            rospy.logerr("something went wrong/")
                            return False
                    
                    return False
                else:
                    self.tool_interface.stop()
                    #fb.feedback.reached_node = node
                    fb.reached_node.data = node
                    self._as.publish_feedback(fb)
                print "SAVE TO MONGO"
                exec_msg.time_of_execution = time.time() - starttime
                exec_msg.covered_distance = self.distance_covered
                self.mongo_utils.insert_in_collection(exec_msg, self.taskid)
            return True
        #LAST TO BE IMPLEMENTED
        elif mode == 2: #GRNavigationActionGoal.VISIT_SOME:
            for n in range(0,len(self.plan),span):
                exec_msg = ExecutionMetadata()
                exec_msg.rowid = self.rowid
                exec_msg.taskid = self.taskid
                exec_msg.action = "RUN"

                self.goal_received = False
                self.goal_finished = False
                print "VISIT_SOME", self.plan[n]

                if self.plan[n] == self.startnode:
                    exec_msg.action = "FREE_MOTION"
                    self.dynconf_client.update_configuration({"constrain_motion": False})
                else:
                    self.dynconf_client.update_configuration({"constrain_motion": True})

                #WAIT FOR MAP UPDATE
                time.sleep(3)
                self.goal = self.nodes_poses[self.plan[n]]
                nav_mode = "FREE_MOTION" if exec_msg.action == "FREE_MOTION" else self.taskid
                self.move_base_server(self.goal, nav_mode)
                if not self.waitMoveBase():
                    return False
                else:
                    #fb.reached_node = node
                    fb.reached_node.data = self.plan[n]
                    self._as.publish_feedback(fb)
                self.mongo_utils.insert_in_collection(exec_msg, self.taskid)
            return Trueg
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

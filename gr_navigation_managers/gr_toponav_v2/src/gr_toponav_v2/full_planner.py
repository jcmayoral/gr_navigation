import tf
import rospy
import networkx as nx
import numpy as np
import yaml
import matplotlib.pyplot as plt
import actionlib
import time

from gr_toponav_v1.states.move_base_state import polyfit_action_mode as polyfit_server
from gr_toponav_v2.mongoutils import MongoManager

import dynamic_reconfigure.client
from gr_action_msgs.msg import GRNavigationAction, GRNavigationActionGoal, GRNavigationActionResult, GRNavigationFeedback, PolyFitRowAction, PolyFitRowResult
from safety_msgs.msg import ExecutionMetadata
from actionlib_msgs.msg import GoalStatus
from visualization_msgs.msg import MarkerArray, Marker
from std_srvs.srv import Trigger, TriggerResponse
from mbf_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalID, GoalStatus, GoalStatusArray


from enum import Enum

class NavResult(Enum):
    FAILURE = 0
    HRI = 1
    SUCCESS = 2

class NavMode(Enum):
    VISIT_ALL = 0
    JUST_END = 1
    VISIT_SOME = 2


class FullTopoPlanner:
    def __init__(self):
        self.temporal_map = None
        self._as = actionlib.SimpleActionServer("gr_simple_manager", GRNavigationAction, execute_cb=self.execute_cb, auto_start = False)
        self._safety_as = rospy.Service("gr_human_intervention/set", Trigger, self.set_safety_human_intervention)
        self._safety_as2 = rospy.Service("gr_human_intervention/reset", Trigger, self.reset_safety_human_intervention)

        #self.action_client = actionlib.SimpleActionClient('polyfit_action', PolyFitRowAction)
        self.action_client = actionlib.SimpleActionClient("move_base_flex/move_base", MoveBaseAction)
        self.load_movebase_params()
        self.goal_received = False
        self.goal_finished = False
        self.mongo_utils = MongoManager()
        self.human_intervention_requested = False

        self.dynconf_client = dynamic_reconfigure.client.Client("topological_to_metric_converter", timeout=20, config_callback=self.config_callback)

        self._as.start()

    def set_safety_human_intervention(self, req):
        rospy.logerr("in safety intervention set")
        self.human_intervention_requested = True#req.data
        return TriggerResponse(success=True, message="Human Intervention required")

    def reset_safety_human_intervention(self, req):
        rospy.logerr("in safety intervention reset ")
        self.human_intervention_requested = False
        return TriggerResponse(success=True, message="Human Intervention required")

    def load_movebase_params(self):
        import rospkg
        r = rospkg.RosPack()
        localpath = r.get_path('gr_toponav_v2')
        with open(localpath + "/config/mbf_config.yaml") as f:
            self.params = yaml.load(f.read(), Loader=yaml.Loader)
        print(self.params)

    def config_callback(self,config):
        pass
        #rospy.loginfo("Config se", config)

    def move_base_server(self, commands, change_row):
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

        if change_row:
            print ("CHANGE ROW or start")
            goal.controller = self.params["CHANGEROW"]["controller"]
            goal.planner = self.params["CHANGEROW"]["planner"]

        else:
            goal.controller = self.params[self.taskid]["controller"]
            goal.planner = self.params[self.taskid]["planner"]

        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        self.lastupdate_time = time.time()
        self.last_pose = None
        self.distance_covered = 0.0
        self.action_client.send_goal(goal, done_cb = self.done_cb, feedback_cb=self.feedback_cb, active_cb=self.active_cb)
        #print "WAITING FOR RESULT"
        #action_client.wait_for_result()
        #print "RESULT GOTTEN "
        #self.action_client.get_status()
        #print "after "
        print (self.action_client.get_result())

        return #action_client.get_result()

    def active_cb(self):
        print ("GOAL received")
        self.goal_received = True

    def calc_distance(self,a,b):
        return np.sqrt(np.power(b[0]-a[0],2)+np.power(b[1]-a[1],2))

    def feedback_cb(self, feedback):
        self.dist2goal = feedback.dist_to_goal
        #print feedback.base_position.header.frame_id, time.time()- self.lastupdate_time
        bp = [feedback.current_pose.pose.position.x, feedback.current_pose.pose.position.y]
        if self.last_pose:
            self.distance_covered +=  self.calc_distance(self.last_pose, bp)
            #rospy.loginfo("Distance covered {} meters".format(self.distance_covered))
            self.lastupdate_time = time.time()

        self.last_pose = bp

        pass
        #print "FB ", feedback

    def done_cb(self, state, result):
        print ("Goal finished with state ", state)
        if state == 0:
            print ("SUCCESS")
            self.distance_covered += self.dist2goal#self.calc_distance(self.goal[:2], self.last_pose)
        if state == 12:
            print ("Collision")
        if state == 16:
            print ("TF ERROR")
        if state == 17:
            print ("internal Error")
        if state == 13:
            print ("oscillation")
        if state == 10:
            print ("failure")
        #print "Goal finished with result ", result
        self.goal_finished = True

    def execute_cb(self, goal):
        result = GRNavigationActionResult()
        result.result.suceeded = False
        if self.human_intervention_requested:
            self._as.set_aborted()
            return

        if self.create_graph(goal.plan.markers):
            print ("MY PLAN from {} to {}".format(goal.start_node, goal.goal_node))
            self.plan = self.get_topological_plan(goal.start_node, goal.goal_node)
            self.startnode = goal.start_node
            self.goalnode = goal.goal_node
            self.rowid = goal.row_id
            self.taskid = goal.task_id
            #TODO SET TRIGGER
            execution_result = self.execute_plan(goal.mode,goal.span)
            print (execution_result)

        if execution_result == NavResult.SUCCESS:
            result.result.suceeded = True
            result.result.intervention = False
            result.result.safety_msg.data = "Normal State"
        elif execution_result == NavResult.FAILURE:
            result.result.suceeded = False
            result.result.intervention = False
            result.result.safety_msg.data = "Navigation Error"
        else:
            result.result.suceeded = False
            result.result.intervention = True
            result.result.safety_msg.data = "Human Intervention Requested"
        #NOT so sure why this crashes
        #self._as.set_succeeded(result)
        if result.result.suceeded:
            self._as.set_succeeded()
            return
        self._as.set_aborted()

    def waitMoveBase(self):
        while not self.goal_finished:
            if self._as.is_preempt_requested():
                print ("Cancel received")
                self.action_client.cancel_all_goals()
                return False
            if self.human_intervention_requested:
                rospy.logerr("cancelling all goals")
                self.action_client.cancel_all_goals()
                return False
            time.sleep(1)
        return True

    def execute_plan(self,mode, span=0):
        print ("THIS IS MY PLAN {} using mode {}".format(self.plan, mode))
        print(mode == NavMode.JUST_END.value, NavMode.JUST_END.value)
        self.goal_received = False
        self.goal_finished = False

        fb = GRNavigationFeedback()
        if mode == NavMode.VISIT_ALL.value:
            for node in self.plan:
                starttime = time.time()
                exec_msg = ExecutionMetadata()
                exec_msg.rowid = self.rowid
                exec_msg.action = "RUN"
                self.goal_received = False
                self.goal_finished = False
                print ("moving to " , node)
                if node == self.startnode:
                    exec_msg.action = "CHANGE_ROW"
                    self.dynconf_client.update_configuration({"constrain_motion": False})
                else:
                    self.dynconf_client.update_configuration({"constrain_motion": True})
                #WAIT FOR MAP UPDATE
                time.sleep(3)
                self.goal = self.nodes_poses[node]
                change_row = True if exec_msg.action == "CHANGE_ROW" else False

                fb.reached_node.data = node
                fb.safety_msg.data = "Robot has been commanded MODE:  " + str(exec_msg.action)
                print ("FB", fb)
                self._as.publish_feedback(fb)
                self.move_base_server(self.goal, change_row)


                if not self.waitMoveBase():
                    return NavResult.HRI if self.human_intervention_requested else NavResult.FAILURE
                else:
                    fb.reached_node.data = node
                    fb.safety_msg.data = "Normal State"
                    #print fb
                    self._as.publish_feedback(fb)

                exec_msg.time_of_execution = time.time() - starttime
                exec_msg.covered_distance = self.distance_covered
                self.mongo_utils.insert_in_collection(exec_msg, self.taskid)

            return True
        elif mode == NavMode.JUST_END.value:
            print("JUST END")
            #LIMIT TO START AND END
            for node in [self.startnode, self.goalnode]:
                exec_msg = ExecutionMetadata()
                starttime = time.time()
                exec_msg.rowid = self.rowid
                exec_msg.taskid = self.taskid
                exec_msg.action = "RUN"

                self.goal_received = False
                self.goal_finished = False

                print ("moving to ", node, " POSE " , self.nodes_poses[node])

                #IF START NODE
                if node == self.startnode:
                    exec_msg.action = "CHANGE_ROW"
                    self.dynconf_client.update_configuration({"constrain_motion": False})
                else:
                    #IF END
                    self.dynconf_client.update_configuration({"constrain_motion": True})

                #WAIT FOR MAP UPDATE
                time.sleep(3)
                self.goal =self.nodes_poses[node]
                change_row = True if exec_msg.action == "CHANGE_ROW" else False


                fb.reached_node.data = node
                fb.safety_msg.data = "Robot has been commanded MODE:  " + str(exec_msg.action)
                print ("FB", fb)
                self._as.publish_feedback(fb)

                self.move_base_server(self.goal, change_row)
                if not self.waitMoveBase():
                    return NavResult.HRI if self.human_intervention_requested else NavResult.FAILURE
                else:
                    #fb.feedback.reached_node = node
                    fb.reached_node.data = node
                    fb.safety_msg.data = "Peforming without errors"
                    fb.executing_time = time.time() - starttime
                    self._as.publish_feedback(fb)
                print ("SAVE TO MONGO")
                exec_msg.time_of_execution = time.time() - starttime
                exec_msg.covered_distance = self.distance_covered
                self.mongo_utils.insert_in_collection(exec_msg, self.taskid)
            return NavResult.SUCCESS

        #LAST TO BE IMPLEMENTED
        elif mode == NavMode.VISIT_SOME.value:
            for n in range(0,len(self.plan),span):
                exec_msg = ExecutionMetadata()
                exec_msg.rowid = self.rowid
                exec_msg.taskid = self.taskid
                exec_msg.action = "RUN"

                self.goal_received = False
                self.goal_finished = False
                print ("VISIT_SOME", self.plan[n])

                if self.plan[n] == self.startnode:
                    exec_msg.action = "CHANGE_ROW"
                    self.dynconf_client.update_configuration({"constrain_motion": False})
                else:
                    self.dynconf_client.update_configuration({"constrain_motion": True})

                #WAIT FOR MAP UPDATE
                time.sleep(3)
                self.goal = self.nodes_poses[self.plan[n]]
                change_row = True if exec_msg.action == "CHANGE_ROW" else False


                fb.reached_node.data = node
                fb.safety_msg.data = "Robot has been commanded MODE:  " + str(exec_msg.action)
                print ("FB", fb)
                self._as.publish_feedback(fb)

                self.move_base_server(self.goal, change_row)
                if not self.waitMoveBase():
                    return NavResult.HRI if self.human_intervention_requested else NavResult.FAILURE
                else:
                    #fb.reached_node = node
                    print (self.plan[n])
                    fb.reached_node.data = self.plan[n]
                    self._as.publish_feedback(fb)
                self.mongo_utils.insert_in_collection(exec_msg, self.taskid)
        else:
            rospy.logerr("ERROR")
            return NavResult.FAILURE



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

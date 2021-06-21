import actionlib
import rospy
import tf
#from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gr_action_msgs.msg import PolyFitRowAction, PolyFitRowGoal


def command_mapper(mode):
    if mode == "move_base":
        return move_base
    if mode == "sbpl_action":
        return sbpl_action_mode

def command_robot_to_node(node_id, no_orientation=True):
    action_client = actionlib.SimpleActionClient('topological_navigation', GotoNodeAction)
    rospy.loginfo("Waiting for Action Server /topological_navigation")
    action_client.wait_for_server()
    rospy.loginfo("Action Server Found")

    rospy.loginfo("Commanding robot to %s", node_id)
    #navgoal = GotoNodeGoal()
    #navgoal.target = node_id
    #navgoal.no_orientation = no_orientation
    #action_client.send_goal(navgoal)
    action_client.wait_for_result()
    rospy.loginfo("Is node %s reached? %r", node_id,  action_client.get_result())
    return action_client.get_result()


def move_base(commands, movebase_server = "move_base"):
    action_client = actionlib.SimpleActionClient(movebase_server, MoveBaseAction)
    rospy.loginfo("Waiting for Action Server "+ movebase_server)
    action_client.wait_for_server()
    rospy.loginfo("Action Server Found")
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
    action_client.send_goal(goal)
    #print "WAITING FOR RESULT"
    #action_client.wait_for_result()
    #print "RESULT GOTTEN "
    #print action_client.get_result()
    #print "after "

    return action_client.get_result()


def move_base2(commands,action_client):
    rospy.loginfo("Waiting for Action Server ")
    action_client.wait_for_server()
    rospy.loginfo("Action Server Found")
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
    print action_client.send_goal(goal)
    #print "WAITING FOR RESULT"
    #action_client.wait_for_result()
    #print "RESULT GOTTEN "
    print action_client.get_status()
    #print "after "

    return #action_client.get_result()


def sbpl_action_mode(commands):
    action_client = actionlib.SimpleActionClient('sbpl_action', MoveBaseAction)
    rospy.loginfo("Waiting for Action Server /sbpl_action")
    action_client.wait_for_server()
    rospy.loginfo("Action Server Found", commands)
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
    action_client.send_goal(goal)
    action_client.wait_for_result()
    return action_client.get_result()
    return action_client.get_result()


def polyfit_action_mode(poses, action_client):
    print "polyfit action mode2"
    goal = PolyFitRowGoal()
    goal.x = poses[:,0]
    goal.y = poses[:,1]
    goal.yaw = poses[:,2]
    action_client.send_goal(goal)
    print "GOAL SENT"
    action_client.wait_for_result()
    print "AAAAAAALLLLLLLLl"


def polyfit_action_modeold(poses):
    print "polyfit action mode"
    action_client = actionlib.SimpleActionClient('polyfit_action', PolyFitRowAction)
    goal = PolyFitRowGoal()
    for p in poses:
        goal.x.append(p[0])
        goal.y.append(p[1])
        goal.yaw.append(p[2])
    print (goal)
    action_client.send_goal(goal)
    action_client.wait_for_result()
    print "AAAAAAALLLLLLLLl"

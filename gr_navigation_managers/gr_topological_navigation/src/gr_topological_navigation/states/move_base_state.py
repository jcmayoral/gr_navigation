import actionlib
import rospy
from topological_navigation.msg import GotoNodeAction, GotoNodeGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


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
    navgoal = GotoNodeGoal()
    navgoal.target = node_id
    navgoal.no_orientation = no_orientation
    action_client.send_goal(navgoal)
    action_client.wait_for_result()
    rospy.loginfo("Is node %s reached? %r", node_id,  action_client.get_result())
    return action_client.get_result()


def move_base(commands):
    action_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    rospy.loginfo("Waiting for Action Server move_base")
    action_client.wait_for_server()
    rospy.loginfo("Action Server Found")
    goal = MoveBaseGoal()
    #to do add orientation
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


def sbpl_action_mode(x =0, y=0, yaw = 0):
    action_client = actionlib.SimpleActionClient('sbpl_action', GotoNodeAction)
    rospy.loginfo("Waiting for Action Server /sbpl_action")
    action_client.wait_for_server()
    rospy.loginfo("Action Server Found")
    goal = MoveBaseGoal()
    #to do add orientation
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.orientation.w = 1.0
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    action_client.send_goal(goal)
    action_client.wait_for_result()
    return action_client.get_result()

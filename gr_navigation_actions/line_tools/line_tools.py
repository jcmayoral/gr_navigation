import rospy
from geometry_msgs.msg import PoseArray, Pose


def create_msg(pub):
    p = PoseArray()
    p.header.stamp = rospy.Time.now()
    p.header.frame_id = "base_link"
    p2 = Pose()
    p2.pose.position.x = 1.0
    p2.pose.position.y = -1.0
    p2.pose.orientation.w = 1.0
    p2.pose.position.y = -1.0
    p.poses.append(p2)
    p2.pose.position.x = 1.0
    p2.pose.position.y = 1.0
    p.poses.append(p2)
    pub.publish(p)


if __name__ == '__main__':
    rospy.init_node("toos")
    pub = rospy.Publisher("/output_line_estimated")
    while not rospy.is_shutdown():
        create_msg(pub)
        rospy.sleep(0.1)



#!/usr/bin/env python3
from gr_toponav_v2.partial_planner import PartialTopoPlanner
import rospy

if __name__ == "__main__":
    rospy.init_node("partial_topoplanner")
    planner = PartialTopoPlanner()
    rospy.spin()

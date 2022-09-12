#!/usr/bin/env python3
from gr_toponav_v2.full_planner import FullTopoPlanner
import rospy 

if __name__ == "__main__":
    rospy.init_node("full_topoplanner")
    planner = FullTopoPlanner()
    rospy.spin()

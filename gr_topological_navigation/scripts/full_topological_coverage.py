#!/usr/bin/python
import rospy
from gr_topological_navigation.topological_planner import TopologicalPlanner

if __name__ == '__main__':
    topological_planner = TopologicalPlanner()
    topological_planner.get_plan_list()

#!/usr/bin/python
import rospy
from gr_topological_navigation.topological_planner import TopologicalPlanner

if __name__ == '__main__':
    topological_planner = TopologicalPlanner()
    topological_planner.generate_full_coverage_plan()

    next_transition = topological_planner.get_next_transition()
    while (next_transition):
        topological_planner.command_robot_to_node(next_transition[1])
        next_transition = topological_planner.get_next_node()

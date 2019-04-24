#!/usr/bin/python
import rospy
from gr_topological_navigation.states.topological_planner import TopologicalPlanner

if __name__ == '__main__':
    rospy.init_node("topological_full_coverage_planner")
    topological_planner = TopologicalPlanner(start_node = "node_0", pointset="trash_map_5")
    topological_planner.go_to_source()

    next_transition = topological_planner.get_next_transition()

    while (next_transition):
        topological_planner.command_robot_to_node(next_transition[1])
        topological_planner.show_statistics()
        next_transition = topological_planner.get_next_transition()

#!/usr/bin/python
import rospy
import sys
from gr_topological_navigation.states.topological_planner import TopologicalPlanner
from gr_topological_navigation.states.move_base_state import command_robot_to_node

if __name__ == '__main__':
    if not sys.argv[1]:
	start_node = "node_0"
    else:
        start_node = sys.argv[1]
    rospy.init_node("topological_full_coverage_planner")
    topological_planner = TopologicalPlanner(start_node = start_node, pointset="wish_map_move_base")
    topological_planner.go_to_source()

    next_transition = topological_planner.get_next_transition()

    while (next_transition):
        command_robot_to_node(next_transition[1])
        topological_planner.current_node = next_transition[1]
        topological_planner.show_statistics()
        next_transition = topological_planner.get_next_transition()

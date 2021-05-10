#include <gr_line_trajectory_planner/line_trajectory_planner.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "line_trajectory_planner_node");
    gr_line_trajectory_planner::GRLinePlanner planner;

    return 0;
}
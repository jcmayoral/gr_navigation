#include <gr_sbpl_trajectory_generator/spbl_planner_wrapper.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "sbpl_trajectory_generator_node");
    gr_sbpl_trajectory_generator::GRSBPLPlanner planner;

    return 0;
}
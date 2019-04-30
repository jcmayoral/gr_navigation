#include <gr_sbpl_trajectory_generator/spbl_planner_wrapper.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "sbpl_trajectory_generator_node");
    gr_sbpl_trajectory_generator::GRSBPLPlanner planner;


    geometry_msgs::PoseStamped start, goal;

    start.header.frame_id = "map";
    start.header.stamp = ros::Time::now();
    goal.header = start.header;


    start.pose.position.x = -15.630;
    start.pose.position.y = -23.987;
    start.pose.orientation.w = 1;

    goal.pose.position.x = -14.630;
    goal.pose.position.y = -22.987;
    goal.pose.orientation.w = 1;


    if (planner.makePlan(start,goal)){
      ROS_INFO("WORKING");
    }
    else{
      ROS_ERROR("ERROR");
    }

    return 0;
}
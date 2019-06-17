#include <gr_safety_policies/proximity_policy.h>

using namespace gr_safety_policies;

int main(int argc, char** argv){
    ros::init(argc, argv, "proximity_policy_node");
    ros::NodeHandle nh;
    ProximityPolicy* policy = new ProximityPolicy();
    policy->instantiateServices(nh);

    while(ros::ok()){
        if(policy->checkPolicy()){
            //ROS_WARN("Proximity monitor activated");
            policy->reportState();
        }
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    return 1;
}

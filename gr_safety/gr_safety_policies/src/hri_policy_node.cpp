#include <gr_safety_policies/hri_policy.h>

using namespace gr_safety_policies;

int main(int argc, char** argv){
    ros::init(argc, argv, "hri_policy_node");
    ros::NodeHandle nh;
    HRIPolicy* policy = new HRIPolicy();
    policy->instantiateServices(nh);

    while(ros::ok()){
        if(policy->checkPolicy()){
            policy->reportState();
        }
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    return 1;
}

#include <gr_safety_policies/policies_manager.h>

using namespace gr_safety_policies;

int main(int argc, char** argv){
    ros::init(argc, argv, "policies_manager_node");
    ros::NodeHandle nh;

    //TODO config file once more monitors type are designed
    PoliciesManager policies_manager;
    policies_manager.addPolicy(new ProximityPolicy());
    
    int i = 0;

    while(ros::ok()){
        policies_manager.monitor();
        ros::spinOnce();
    }
    return 1;
}

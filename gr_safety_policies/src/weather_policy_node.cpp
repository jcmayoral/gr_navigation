#include <gr_safety_policies/weather_policy.h>

using namespace gr_safety_policies;

int main(int argc, char** argv){
    ros::init(argc, argv, "weather_policy_node");
    ros::NodeHandle nh;
    WeatherPolicy* policy = new WeatherPolicy();
    policy->instantiateServices(nh);

    while(ros::ok()){
        if(policy->detectFault()){
            //ROS_WARN("Proximity monitor activated");
            policy->isolateFault();
        }
        //ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    return 1;
}
#include <gr_safety_monitors/proximity_monitor.h>

using namespace gr_safety_monitors;

int main(int argc, char** argv){
    ros::init(argc, argv, "proximity_monitor_node");
    ros::NodeHandle nh;
    ProximityMonitor* monitor = new ProximityMonitor();
    monitor->instantiateServices(nh);

    int i = 0;

    while(ros::ok()){
        if(monitor->detectFault()){
            ROS_WARN("Proximity monitor activated");
            monitor->isolateFault();
        }
        if (i%10){
            monitor->publishTopics();
            i=0;
        }
        ++i;
        ros::spinOnce();
    }
    return 1;
}
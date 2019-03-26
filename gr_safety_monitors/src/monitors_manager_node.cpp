#include <gr_safety_monitors/monitors_manager.h>

using namespace gr_safety_monitors;

int main(int argc, char** argv){
    ros::init(argc, argv, "proximity_monitor_node");
    ros::NodeHandle nh;

    //TODO config file once more monitors type are designed
    MonitorsManager monitor_manager;

    //ProximityMonitor* monitor;
    
    /*
     = new ProximityMonitor();
    monitor->instantiateServices(nh);

    int i = 0;

    while(ros::ok()){
        if(monitor->detectFault()){
            //ROS_WARN("Proximity monitor activated");
            monitor->isolateFault();
        }
        if (i%10){
            monitor->publishTopics();
            i=0;
        }
        ++i;
        ros::Duration(0.5).sleep();
        ros::spinOnce();
    }
    */
    return 1;
}
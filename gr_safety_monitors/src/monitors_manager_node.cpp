#include <gr_safety_monitors/monitors_manager.h>

using namespace gr_safety_monitors;

int main(int argc, char** argv){
    ros::init(argc, argv, "monitors_manager_node");
    ros::NodeHandle nh;

    //TODO config file once more monitors type are designed
    MonitorsManager monitor_manager;
    monitor_manager.addMonitor(new ProximityMonitor());
    
    int i = 0;

    while(ros::ok()){
        monitor_manager.monitor();
        ros::spinOnce();
    }
    return 1;
}

#include <gr_safety_monitors/monitors_manager.h>


using namespace gr_safety_monitors;


MonitorsManager::MonitorsManager(): nh_(){

}

MonitorsManager::~MonitorsManager(){

}

void MonitorsManager::addMonitor(fault_core::FaultDetector* new_monitor){
    new_monitor->instantiateServices(nh_);
    monitors_.push_back(new_monitor);
}

void MonitorsManager::monitor(){
    for (auto i=0; i< monitors_.size(); i++){
        if(monitors_[i]->detectFault()){
            //ROS_WARN("Proximity monitor activated");
            monitors_[i]->isolateFault();
        }

        //monitors_[i]->publishTopics();
    }
    
}

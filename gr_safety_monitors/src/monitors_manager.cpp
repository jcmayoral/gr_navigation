#include <gr_safety_monitors/monitors_manager.h>


using namespace gr_safety_monitors;


MonitorsManager::MonitorsManager(){

}

MonitorsManager::~MonitorsManager(){

}

void MonitorsManager::addMonitor(fault_core::FaultDetector* new_monitor){
    monitors_.push_back(new_monitor);
}

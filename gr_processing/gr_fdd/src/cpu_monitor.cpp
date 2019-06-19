#include <gr_fdd/cpu_monitor.h>

CPUMonitor::CPUMonitor(): last_cpu_usage_(1.0), last_cpu_total_(1.0) {
      cpu_usage_.resize(10);
      ROS_INFO("CPUMonitor constructor");
}

CPUMonitor::CPUMonitor(const CPUMonitor& orig) {
}

void CPUMonitor::updateData(int index, double value){
    cpu_usage_[index] = value;
}

CPUMonitor::~CPUMonitor() {
  
}

double CPUMonitor::getUsage(){
    
    double cpu_usage = 0.0;
    double current_cpu_usage = (getActiveTime());
    double total_cpu = std::accumulate(cpu_usage_.begin(), cpu_usage_.end(),0.0);
                                       
    double work_overperiod = last_cpu_usage_ - current_cpu_usage;
    double total_cpu_period = last_cpu_total_ - total_cpu;
                    
    cpu_usage = 100*work_overperiod/total_cpu_period;
    last_cpu_usage_ = current_cpu_usage;
    last_cpu_total_ = total_cpu;
    return cpu_usage;
}

double CPUMonitor::getActiveTime()
{
    //FROM http://blog.davidecoppola.com/2016/12/cpp-program-to-get-cpu-usage-from-command-line-in-linux/
    return cpu_usage_[0] +
           cpu_usage_[1] +
           cpu_usage_[2] +
           cpu_usage_[5] +
           cpu_usage_[6] +
           cpu_usage_[7] +
           cpu_usage_[8] +
           cpu_usage_[9];
}

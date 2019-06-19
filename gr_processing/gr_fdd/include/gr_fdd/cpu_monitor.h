#include<vector>
#include<numeric>
#include<fstream>
#include <ros/ros.h>

#ifndef CPUMONITOR_H
#define CPUMONITOR_H

class CPUMonitor {
public:
    CPUMonitor();
    CPUMonitor(const CPUMonitor& orig);
    virtual ~CPUMonitor();
    double getUsage();
    void updateData(int index, double value);

protected:
    double getActiveTime();

private:
    double last_cpu_usage_;
    double last_cpu_total_;
    std::vector <double> cpu_usage_;
    
};

#endif /* CPUMONITOR_H */


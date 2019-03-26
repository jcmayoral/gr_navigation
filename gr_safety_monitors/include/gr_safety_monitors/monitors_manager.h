#include <fault_core/fault_detector.h>
#include <gr_safety_monitors/proximity_monitor.h>

#include <vector>

namespace gr_safety_monitors
{
    class MonitorsManager{
        public:
            MonitorsManager();
            ~MonitorsManager();
            void addMonitor(fault_core::FaultDetector* new_monitor);
            void monitor();
        private:
            std::vector<fault_core::FaultDetector*> monitors_;
            ros::NodeHandle nh_;
    };
}
#ifndef POLICIES_MANAGER_H
#define POLICIES_MANAGER_H

#include <safety_core/fault_detector.h>
#include <gr_safety_policies/proximity_policy.h>

#include <vector>

namespace gr_safety_policies
{
    class PoliciesManager{
        public:
            PoliciesManager();
            ~PoliciesManager();
            void addPolicy(safety_core::FaultDetector* new_policy);
            void monitor();
        private:
            std::vector<safety_core::FaultDetector*> policies_;
            ros::NodeHandle nh_;
    };
};

#endif

#ifndef POLICIES_MANAGER_H
#define POLICIES_MANAGER_H

#include <safety_core/safe_policy.h>
#include <gr_safety_policies/proximity_policy.h>

#include <vector>

namespace gr_safety_policies
{
    class PoliciesManager{
        public:
            PoliciesManager();
            ~PoliciesManager();
            void addPolicy(safety_core::SafePolicy* new_policy);
            void monitor();
        private:
            std::vector<safety_core::SafePolicy*> policies_;
            ros::NodeHandle nh_;
    };
};

#endif

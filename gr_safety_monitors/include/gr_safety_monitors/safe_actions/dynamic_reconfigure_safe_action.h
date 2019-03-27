#include <gr_safety_monitors/safe_actions/safe_action.h>
#include <ros/ros.h>

namespace gr_safety_monitors{
    class DynamicReconfigureSafeAction : public SafeAction{
        public:
            DynamicReconfigureSafeAction();
            ~DynamicReconfigureSafeAction();
            virtual void execute();
            virtual void stop();
    };
};
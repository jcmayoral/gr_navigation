#ifndef DYNAMIC_RECONFIGURE_SAFE_ACTION_H
#define DYNAMIC_RECONFIGURE_SAFE_ACTION_H

#include <gr_safety_monitors/safe_actions/safe_action.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>

namespace gr_safety_monitors{
    class DynamicReconfigureSafeAction : public SafeAction{
        public:
            DynamicReconfigureSafeAction();
            ~DynamicReconfigureSafeAction();
            virtual void execute();
            virtual void stop();
    };  
};

#endif
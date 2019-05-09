#ifndef DYNAMIC_RECONFIGURE_SAFE_ACTION_H
#define DYNAMIC_RECONFIGURE_SAFE_ACTION_H

#include <safety_core/safe_action.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/BoolParameter.h>

using namespace safety_core;

namespace gr_safety_policies{
    class DynamicReconfigureSafeAction : public SafeAction{
        public:
            DynamicReconfigureSafeAction();
            ~DynamicReconfigureSafeAction();
            virtual void execute();
            virtual void stop();
    };
};

#endif

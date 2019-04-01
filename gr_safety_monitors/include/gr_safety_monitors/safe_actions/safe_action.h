#ifndef SAFE_ACTION_H
#define SAFE_ACTION_H

#include <ros/ros.h>

namespace gr_safety_monitors{
    class SafeAction{
        public:
            SafeAction(){
            };
            virtual ~SafeAction(){};
            virtual void execute()=0;
            virtual void stop()=0;
    };
};

#endif
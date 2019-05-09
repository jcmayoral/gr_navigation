#ifndef SAFE_ACTION_H
#define SAFE_ACTION_H

#include <ros/ros.h>

namespace safety_core{
    class SafeAction{
        public:
            SafeAction(){
            };
            virtual ~SafeAction(){};
            virtual void execute()=0;
            virtual void stop()=0;
            virtual int getSafetyID(){
              return safety_id_;
            }
            int safety_id_ = -1;
    };
};

#endif

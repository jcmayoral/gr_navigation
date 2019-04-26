#ifndef PUBLISHER_SAFE_ACTION_H
#define PUBLISHER_SAFE_ACTION_H

#include <gr_safety_policies/safe_actions/safe_action.h>
#include <std_msgs/Bool.h>
#include <ros/ros.h>

namespace gr_safety_policies{
    class PublisherSafeAction : public SafeAction{
        public:
            PublisherSafeAction();
            ~PublisherSafeAction();
            virtual void execute();
            virtual void stop();
        protected:
            ros::Publisher topic_publisher_;
    };
};

#endif

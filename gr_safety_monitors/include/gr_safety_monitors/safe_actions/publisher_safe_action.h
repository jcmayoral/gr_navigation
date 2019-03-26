#include <gr_safety_monitors/safe_actions/safe_action.h>
#include <ros/ros.h>

namespace gr_safety_monitors{
    class PublisherSafeAction : public SafeAction{
        public:
            PublisherSafeAction();
            ~PublisherSafeAction();
            virtual void execute();
    };
};
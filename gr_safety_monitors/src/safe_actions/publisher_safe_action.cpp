#include <gr_safety_monitors/safe_actions/publisher_safe_action.h>

using namespace gr_safety_monitors;

PublisherSafeAction::PublisherSafeAction(){
    ROS_INFO("Constructor Publisher SafeAction");
};

PublisherSafeAction::~PublisherSafeAction(){
};

void PublisherSafeAction::execute(){
    ROS_WARN("Executing  Publisher SafeAction");
};
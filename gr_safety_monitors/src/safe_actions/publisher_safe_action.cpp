#include <gr_safety_monitors/safe_actions/publisher_safe_action.h>

using namespace gr_safety_monitors;

PublisherSafeAction::PublisherSafeAction(){
    //ROS_INFO("Constructor Publisher SafeAction");
    safety_id_ = 0;
    ros::NodeHandle nh;
    topic_publisher_ = nh.advertise<std_msgs::Bool>("lock_all", 1);
};

PublisherSafeAction::~PublisherSafeAction(){
    //ROS_INFO("Destructor");
    //stop();
};

void PublisherSafeAction::execute(){
    ROS_INFO_THROTTLE(5,"Executing  Publisher SafeAction");
    std_msgs::Bool topic;
    topic.data = true;
    topic_publisher_.publish(topic);
};

void PublisherSafeAction::stop(){
    ROS_WARN("Stop  Publisher SafeAction");
    std_msgs::Bool topic;
    topic.data = false;
    topic_publisher_.publish(topic);
};

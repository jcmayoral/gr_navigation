#include <gr_safety_policies/safe_actions/publisher_safe_action.h>
#include <pluginlib/class_list_macros.h>

//PLUGINLIB_DECLARE_CLASS(gr_safety_policies, PublisherSafeAction,
  //                      gr_safety_policies::PublisherSafeAction,
    //                    safety_core::SafeAction)

PLUGINLIB_EXPORT_CLASS(gr_safety_policies::PublisherSafeAction,safety_core::SafeAction)
using namespace gr_safety_policies;

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
    ROS_ERROR("Executing  Stop Publisher SafeAction");
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

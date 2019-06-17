#include "gr_safety_policies/hri_policy.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_DECLARE_CLASS(gr_safety_policies, HRIPolicy,
                        gr_safety_policies::HRIPolicy,
                        safety_core::SafePolicy)

using namespace safety_core;


namespace gr_safety_policies
{
    HRIPolicy::HRIPolicy(){
        ROS_INFO("HRI Policy initialized");
    }

    HRIPolicy::~HRIPolicy(){

    }

    void HRIPolicy::instantiateServices(ros::NodeHandle nh){

    }

    bool HRIPolicy::checkPolicy(){
        return false;
    }

    void HRIPolicy::suggestAction(){

    }

    void HRIPolicy::commands_CB(const std_msgs::String::ConstPtr& command){

    }
}
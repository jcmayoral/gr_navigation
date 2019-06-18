#include "gr_safety_policies/hri_policy.h"
#include <pluginlib/class_list_macros.h>
#include <iostream>

PLUGINLIB_DECLARE_CLASS(gr_safety_policies, HRIPolicy,
                        gr_safety_policies::HRIPolicy,
                        safety_core::SafePolicy)

using namespace safety_core;


namespace gr_safety_policies
{
    HRIPolicy::HRIPolicy(): action_loader_("safety_core", "safety_core::SafeAction"),
                            is_action_executed_(false), is_stop_requested_(false),
                            is_action_requested_(false){
        loadActionClasses();
        policy_.id_ = "HRI_POLICY";
        policy_.action_ =  -1;
        policy_.state_ = PolicyDescription::UNKNOWN;
        policy_.type_ = PolicyDescription::ACTIVE;
        ROS_INFO("HRI Policy initialized");
    }

    HRIPolicy::~HRIPolicy(){

    }

    void HRIPolicy::loadActionClasses(){
        action_classes_ = action_loader_.getDeclaredClasses();
    }

    void HRIPolicy::instantiateServices(ros::NodeHandle nh){
        command_sub_ = nh.subscribe("commands", 1, &HRIPolicy::commands_CB, this);
    }

    bool HRIPolicy::checkPolicy(){
        return is_action_requested_;
    }

    void HRIPolicy::suggestAction(){
        if (!is_action_executed_){
            current_action_->execute();
            policy_.state_ = PolicyDescription::UNSAFE;
            is_action_executed_ = true;
        }

        if (is_stop_requested_){
            current_action_->stop();
            policy_.state_ = PolicyDescription::SAFE;
            is_action_executed_ = false;
            is_action_requested_ = false;
            is_stop_requested_ = false;
        }

    }

    void HRIPolicy::commands_CB(const std_msgs::String::ConstPtr& command){
        if (command->data.empty()){
          ROS_WARN("Empty command string...ignoring");
          return;
        }
        for (int i=0; i < action_classes_.size(); i++){
            if (boost::contains(action_classes_[i], command->data.c_str())){
                instantiateRequestedAction(action_classes_[i]);
                return;
            }
        }
    }

    void HRIPolicy::instantiateRequestedAction(std::string desired_action){
        try{
             if (is_action_executed_){
                 is_stop_requested_ =true;
                 ROS_INFO("STOP ACTION");
                 return;
             }
             if(action_loader_.isClassAvailable(desired_action)){
                 ROS_INFO("Available Action... Loading");
                 current_action_ = action_loader_.createInstance(desired_action.c_str());
                 ROS_INFO("Created safe_action %s", desired_action.c_str());
                 is_action_requested_ = true;
             }
        }
         catch (const pluginlib::PluginlibException& ex){
             ROS_FATAL("Failed to create action %s", ex.what());
             exit(1);
        }

    }
}

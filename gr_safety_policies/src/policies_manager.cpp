#include <gr_safety_policies/policies_manager.h>


using namespace gr_safety_policies;


PoliciesManager::PoliciesManager(): nh_(){

}

PoliciesManager::~PoliciesManager(){

}

void PoliciesManager::addPolicy(safety_core::FaultDetector* new_policy){
    new_policy->instantiateServices(nh_);
    policies_.push_back(new_policy);
}

void PoliciesManager::monitor(){
    for (auto i=0; i< policies_.size(); i++){
        if(policies_[i]->detectFault()){
            //ROS_WARN("Proximity monitor activated");
            policies_[i]->isolateFault();
        }

        //monitors_[i]->publishTopics();
    }
    
}

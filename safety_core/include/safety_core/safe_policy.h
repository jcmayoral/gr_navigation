#ifndef SAFE_POLICY_H_
#define SAFE_POLICY_H_
#include <safety_core/policy_description.h>

namespace safety_core {
  class SafePolicy{
    public:

      virtual void instantiateServices(ros::NodeHandle nh) = 0;
      virtual bool checkPolicy() = 0;
      virtual void reportState(){
          if (policy_.getType() == PolicyDescription::ACTIVE){
            suggestAction();
          }
          else{
            std::cout << "Policy not active what should passive policies do?" << std::endl;
          }
        }
      virtual void suggestAction() = 0;
      virtual PolicyDescription getPolicyDescription(){
        return policy_;
      };
      virtual ~SafePolicy(){}

    protected:
      PolicyDescription policy_;
  };
};
#endif

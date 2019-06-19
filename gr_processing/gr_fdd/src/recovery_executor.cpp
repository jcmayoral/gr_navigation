#include <gr_fdd/recovery_executor.h>

using namespace gr_fdd;

RecoveryExecutor::RecoveryExecutor(YAML::Node node) {
    std::cout << "RE Constructor" << std::endl;
    for (YAML::const_iterator a= node.begin(); a != node.end(); ++a){
        std::string name = a->first.as<std::string>();
        YAML::Node config = a->second;
        strategy_selector_[name] = config["strategy"].as<std::string>();
    }
}

std::string RecoveryExecutor::getRecoveryStrategy(std::string key){
    if (strategy_selector_[key].empty()){
        return "Unknown Error";
    }
    return strategy_selector_[key];    
}


RecoveryExecutor::~RecoveryExecutor() {
}


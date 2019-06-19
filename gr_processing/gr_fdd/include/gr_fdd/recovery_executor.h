#ifndef RECOVERY_EXECUTOR_H
#define RECOVERY_EXECUTOR_H

#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>

namespace gr_fdd{
    class RecoveryExecutor {
    public:
        RecoveryExecutor(YAML::Node node);
        RecoveryExecutor(const RecoveryExecutor& orig, YAML::Node node) {
        }
        virtual ~RecoveryExecutor();
        std::string getRecoveryStrategy(std::string key);
    private:
        std::map<std::string, std::string> strategy_selector_;
    };
};
#endif /* RECOVERY_EXECUTOR_H */


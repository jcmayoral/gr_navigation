#ifndef FAULT_TOPOLOGY_H
#define FAULT_TOPOLOGY_H

#include <ros/ros.h>

namespace safety_core {
  class PolicyDescription {
    public:
      enum State {
        SAFE,
        UNSAFE,
        UNKNOWN
      };

    std::string id_;
    int action_;
    State state_;
  };
};

#endif

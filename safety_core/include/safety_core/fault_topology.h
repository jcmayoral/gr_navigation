#ifndef FAULT_TOPOLOGY_H
#define FAULT_TOPOLOGY_H

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>

namespace safety_core {
  class FaultTopology {
    public:
      enum FaultType {
        COLLISION,
        SENSORFAULT,
        UNKNOWN_TYPE
      };

      enum FaultCause {
        MISLOCALIZATION,
        DYNAMIC_OBSTACLE,
        MAP_UNACCURACY,
        STATIC_OBSTACLE,
        UNKNOWN
      };
    FaultType type_;
    FaultCause cause_;
    geometry_msgs::Quaternion orientation_;
    double strength_;
  };
};

#endif

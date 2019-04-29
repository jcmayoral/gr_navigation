#ifndef GR_SBPL_PLANNER_H
#define GR_SBPL_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

/** ROS **/
#include <ros/ros.h>

// Costmap used for the map representation
#include <costmap_2d/costmap_2d_ros.h>

// sbpl headers
#include <sbpl/headers.h>

namespace gr_sbpl_trajectory_generator{
    class GRSBPLPlanner{
        public:
            GRSBPLPlanner();
            ~GRSBPLPlanner();
    };
};

#endif

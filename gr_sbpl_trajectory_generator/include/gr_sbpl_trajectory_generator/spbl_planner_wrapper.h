#ifndef GR_SBPL_PLANNER_H
#define GR_SBPL_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/footprint.h>

// sbpl headers
#include <sbpl/headers.h>

namespace gr_sbpl_trajectory_generator{
    class GRSBPLPlanner{
        public:
            GRSBPLPlanner();
            ~GRSBPLPlanner();
            unsigned char costMapCostToSBPLCost(unsigned char newcost);
        private:
            SBPLPlanner* planner_;
            EnvironmentNAVXYTHETALAT* env_;
            nav_msgs::OccupancyGrid costmap_;
            ros::NodeHandle nh_;
            std::string primitive_filename_;
            unsigned char sbpl_cost_multiplier_;
            unsigned char lethal_obstacle_;
            unsigned char inscribed_inflated_obstacle_;
    };
};

#endif

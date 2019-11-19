#ifndef GR_SBPL_PLANNER_H
#define GR_SBPL_PLANNER_H

#include <iostream>
#include <vector>

using namespace std;

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/footprint.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <std_msgs/Float32.h>
#include <actionlib/server/simple_action_server.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

// sbpl headers
#include <sbpl/headers.h>

namespace gr_sbpl_trajectory_generator{
    class GRSBPLPlanner{
        public:
            GRSBPLPlanner();
            ~GRSBPLPlanner();
            unsigned char costMapCostToSBPLCost(unsigned char newcost);
            bool makePlan(geometry_msgs::PoseStamped start,
                          geometry_msgs::PoseStamped goal);
            void point_cb(const geometry_msgs::PointStampedConstPtr msg);
            void executeCB(const move_base_msgs::MoveBaseGoalConstPtr &goal);
            bool executePath();
            void odom_cb(const nav_msgs::OdometryConstPtr odom_msg);
            void setStart();
            void stop();
            double getRotationInFrame(geometry_msgs::PoseStamped& pose, std::string frame);

        private:
            std::vector<geometry_msgs::PoseStamped> plan_;
            bool odom_received_;
            nav_msgs::Odometry odom_msg_;
            //same as carrot action and actionlib tutorial
            actionlib::SimpleActionServer<move_base_msgs::MoveBaseAction>* as_;
            std::string action_name_;
            move_base_msgs::MoveBaseFeedback feedback_;
            move_base_msgs::MoveBaseResult result_;
            ros::Publisher cmd_vel_pub_;

            SBPLPlanner* planner_;
            EnvironmentNAVXYTHETALAT* env_;
            nav_msgs::OccupancyGrid costmap_;
            boost::shared_ptr<nav_msgs::MapMetaData const> map_metadata_;
            ros::NodeHandle nh_;
            std::string primitive_filename_;
            unsigned char sbpl_cost_multiplier_;
            unsigned char lethal_obstacle_;
            unsigned char inscribed_inflated_obstacle_;
            double initial_epsilon_;
            double allocated_time_;
            double position_tolerance_;
            ros::Publisher plan_pub_;
            ros::Publisher time_pub_;
            ros::Subscriber point_sub_;
            ros::Subscriber odom_sub_;
            geometry_msgs::PoseStamped start_;
            geometry_msgs::PoseStamped goal_;
            bool is_start_received_;
            tf2_ros::Buffer tfBuffer;
            tf2_ros::TransformListener tf2_listener;

    };
};

#endif

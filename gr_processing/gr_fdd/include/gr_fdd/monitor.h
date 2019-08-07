#include <ros/ros.h>
#include <gr_fdd/cpu_monitor.h>
#include <topic_tools/shape_shifter.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <gr_fdd/data_container.h>
#include <gr_fdd/recovery_executor.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <sensor_msgs/NavSatFix.h>
#include <yaml-cpp/yaml.h>
#include <ros/package.h>
#include <string>
#include <mutex>
#include <fstream>
#include <unistd.h>

#ifndef MAINMONITOR_H
#define MAINMONITOR_H

class MainMonitor {
public:

    //TODO IDEA Taken from Cartographer

    template <typename MessageType>
     ::ros::Subscriber SubscribeWithHandler(
                            void (MainMonitor::* handler) (int, const std::string&, const typename MessageType::ConstPtr&),
                            const int index,
                            const std::string& config_file,const std::string& topic,
                            ::ros::NodeHandle* const node_handle, 
                            MainMonitor* const node) {
        
            return node_handle->subscribe<MessageType>(
                                        topic, 5,
                                        boost::function<void(const typename MessageType::ConstPtr&)>(
                                                    [node, handler, config_file,index, topic] (const typename MessageType::ConstPtr& msg) 
                                                    {            
                                                        (node->*handler)(msg, index); 
                                                    }));
    }


    MainMonitor(std::string config_file="config/default_config.yml");
    MainMonitor(const MainMonitor& orig);
    void main_cb();
    void in_cb(const topic_tools::ShapeShifter::ConstPtr& msg, int index, std::string topic_name);
    virtual ~MainMonitor();
    // TODO CREATE TEMPLATE
    void empty_cb(const std_msgs::EmptyConstPtr msg, int index);
    void map_cb(const nav_msgs::OccupancyGridConstPtr msg, int index);
    void twist_cb(const geometry_msgs::TwistConstPtr msg, int index);
    void odom_cb(const nav_msgs::OdometryConstPtr msg, int index);
    void imu_cb(const sensor_msgs::ImuConstPtr msg, int index);
    void joints_cb(const sensor_msgs::JointStateConstPtr msg, int index);
    void float_cb(const std_msgs::Float64ConstPtr msg, int index);
    void fix_cb(const sensor_msgs::NavSatFixConstPtr msg, int index);

    void detect_faults(const ros::TimerEvent&);
    std::string isolate_components(std::list<std::string> error_topics);
    double readStatsCPU();

private:
    std::vector<ros::Subscriber> main_subscriber_;
    std::vector<gr_fdd::DataContainer> data_containers_;
    ros::NodeHandle node;
    ros::Timer timer_;
    std::string config_file_;
    CPUMonitor cpu_monitor_;
    ros::Publisher monitor_status_pub_;
    ros::Publisher monitor_diagnostic_pub_;
    ros::Publisher strategy_pub_;
    gr_fdd::RecoveryExecutor* recovery_executor_;
    bool fault_detected_;
};

#endif /* MAINMONITOR_H */


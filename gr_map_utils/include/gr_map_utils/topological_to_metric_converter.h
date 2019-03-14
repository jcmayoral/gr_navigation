#include <gr_map_utils/map_converter_interface.h>
#include <boost/foreach.hpp>
#include <mutex>
#include <vector>
#include <strands_navigation_msgs/TopologicalMap.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>

namespace gr_map_utils{
    
    class Topological2MetricMap : public MapConverterInterface{
        public:
            Topological2MetricMap(ros::NodeHandle nh);
            ~Topological2MetricMap();
            virtual bool storeMap();
            virtual bool getMap();
            virtual void getMapFromTopic();
            virtual void transformMap();
            virtual void publishMaps();
        private:
            strands_navigation_msgs::TopologicalMap topological_map_;
            nav_msgs::OccupancyGrid created_map_;        
            ros::Subscriber topological_map_sub_;
            ros::Publisher map_pub_;
            ros::Publisher metadata_pub_;
            ros::NodeHandle nh_;
            std::mutex mutex_;
            tf2_ros::Buffer tf_buffer_;
            tf2_ros::TransformListener tf2_listener_;
    };
}

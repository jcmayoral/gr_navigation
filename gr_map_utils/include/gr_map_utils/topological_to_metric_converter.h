#include <gr_map_utils/map_converter_interface.h>
#include <gr_map_utils/tf_frame_publisher.h>

#include <mongodb_store/message_store.h>
#include <boost/foreach.hpp>
#include <mutex>
#include <vector>
#include <strands_navigation_msgs/TopologicalNode.h>
#include <strands_navigation_msgs/TopologicalMap.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

namespace gr_map_utils{
    
    class Topological2MetricMap : public MapConverterInterface{
        public:
            Topological2MetricMap(ros::NodeHandle nh);
            ~Topological2MetricMap();
            virtual bool storeMap();
            virtual bool getMap();
            void getMapFromTopic();
            void convertTopologicalMap();
            void publishMaps();
        private:
    		mongodb_store::MessageStoreProxy* message_store_;
            strands_navigation_msgs::TopologicalMap topological_map_;
            nav_msgs::OccupancyGrid created_map_;        
            ros::Subscriber topological_map_sub_;
            ros::Publisher map_pub_;
            ros::Publisher metadata_pub_;
            ros::NodeHandle nh_;
            std::mutex mutex_;

            TfFramePublisher* gr_tf_publisher_;
    };
}

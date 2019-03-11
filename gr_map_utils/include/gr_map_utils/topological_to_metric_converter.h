#include <gr_map_utils/map_converter_interface.h>
#include <mongodb_store/message_store.h>
#include <boost/foreach.hpp>
#include <strands_navigation_msgs/TopologicalNode.h>
#include <strands_navigation_msgs/TopologicalMap.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>

#include <vector>

namespace gr_map_utils{
    class Topological2MetricMap : public MapConverterInterface{
        public:
            Topological2MetricMap(ros::NodeHandle nh);
            ~Topological2MetricMap();
            virtual bool storeMap();
            virtual bool getMap();
            void getMapFromTopic();
        private:
    		mongodb_store::MessageStoreProxy* message_store_;
            strands_navigation_msgs::TopologicalMap topological_map_;
            ros::Subscriber topological_map_sub_;
            ros::NodeHandle nh_;
    };
}

#include <gr_map_utils/map_converter_interface.h>
#include <mongodb_store/message_store.h>
#include <strands_navigation_msgs/TopologicalNode.h>
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
        private:
    		mongodb_store::MessageStoreProxy* message_store_;
            strands_navigation_msgs::TopologicalNode topological_map_;
    };
}

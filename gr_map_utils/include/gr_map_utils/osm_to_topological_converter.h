#include <gr_map_utils/map_converter_interface.h>
#include <boost/foreach.hpp>
#include <mutex>
#include <vector>

#include <strands_navigation_msgs/TopologicalMap.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

namespace gr_map_utils{
    
    class Osm2TopologicalMap : public MapConverterInterface{
        public:
            Osm2TopologicalMap(ros::NodeHandle nh);
            ~Osm2TopologicalMap();
            virtual bool storeMap();
            virtual bool getMap();
            virtual void getMapFromTopic();
            virtual void transformMap();
            virtual void publishMaps();

            void osm_map_cb(const visualization_msgs::MarkerArray::ConstPtr& map);
        private:
            visualization_msgs::MarkerArray osm_map_;
            visualization_msgs::MarkerArray filtered_map_;
            strands_navigation_msgs::TopologicalMap topological_map_;
            ros::Publisher topological_map_pub_;
            ros::Publisher topological_marker_pub_;
            ros::Subscriber osm_map_sub_;
            ros::NodeHandle nh_;
            std::mutex mutex_;
    };
}

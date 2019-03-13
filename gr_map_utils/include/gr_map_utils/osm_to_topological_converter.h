#include <gr_map_utils/map_converter_interface.h>
#include <boost/foreach.hpp>
#include <mutex>

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
        private:
            visualization_msgs::MarkerArray osm_map_;
            ros::Publisher topological_map_pub_;
            std::mutex mutex_;
    };
}

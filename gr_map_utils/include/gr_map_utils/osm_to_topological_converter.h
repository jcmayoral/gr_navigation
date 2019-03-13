#include <gr_map_utils/map_converter_interface.h>
#include <ros/ros.h>

namespace gr_map_utils{
    class Osm2TopologicalMap : public MapConverterInterface{
        public:
            Osm2TopologicalMap(ros::NodeHandle nh);
            ~Osm2TopologicalMap();
            virtual bool storeMap();
            virtual bool getMap();
        private:
            ros::Subscriber map_cb_;
    };
}

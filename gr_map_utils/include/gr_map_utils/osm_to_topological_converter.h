#include <gr_map_utils/map_converter_interface.h>
#include <ros/ros.h>

namespace gr_map_utils{
    class Osm2TopologicalMap : public MapConverterInterface{
        public:
            Osm2TopologicalMap();
            ~Osm2TopologicalMap();
            virtual bool storeMap();
            virtual bool getMap();
    };
}

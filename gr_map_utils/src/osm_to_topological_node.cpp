#include <gr_map_utils/osm_to_topological_converter.h>
using namespace gr_map_utils;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "osm_to_topological_converter");
    ros::NodeHandle nh;
    Osm2TopologicalMap map_converter(nh);

    ros::Rate loop_rate(10);

    if (false){ //osm publish on different frecuency
        map_converter.getMapFromTopic();
    }

    while (ros::ok()){
        loop_rate.sleep();
        map_converter.transformMap();
        map_converter.publishMaps();
        ros::spinOnce();
    }

    return 0;
}

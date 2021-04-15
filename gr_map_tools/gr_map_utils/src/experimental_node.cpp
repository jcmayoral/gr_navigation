#include <gr_map_utils/osm_to_metric_converter.h>
using namespace gr_map_utils;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "osm_to_metric_experimental");
    ros::NodeHandle nh;
    Osm2MetricMap map_converter(nh, "/visualization_marker_array", "map","buildings_osm", true);
    Osm2MetricMap map_converter_2(nh, "/temporal_topological_map","map2", "edges", false, "line");

    ros::Rate loop_rate(10);


    if (!map_converter.getMap()){
        ROS_ERROR("Map not gotten");
        return 1;
    }
    if (!map_converter_2.getMap()){
        ROS_ERROR("Map2 not gotten");
        return 1;
    }

    map_converter.transformMap();
    map_converter_2.transformMap();

    //map_converter.storeMap();

    while (ros::ok()){
        loop_rate.sleep();
        map_converter.publishMaps();
        map_converter_2.publishMaps();
        ros::spinOnce();
    }

    return 0;
}

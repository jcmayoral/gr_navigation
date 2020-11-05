#include <gr_map_utils/osm_to_metric_converter.h>
using namespace gr_map_utils;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "osm_to_metric_converter");
    ros::NodeHandle nh;
    Osm2MetricMap map_converter(nh);

    ros::Rate loop_rate(10);

    
    if (!map_converter.getMap()){
        ROS_ERROR("Map not gotten");
        return 1;
    }

    map_converter.transformMap();
    map_converter.storeMap();

    while (ros::ok()){
        loop_rate.sleep();
        map_converter.publishMaps();
        ros::spinOnce();
    }

    return 0;
}
